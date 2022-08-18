#include <l3_footstep_planning_libs/modeling/step_plan.h>

#include <tf_conversions/tf_eigen.h>

#include <l3_libs/conversions/l3_msg_conversions.h>

namespace l3_footstep_planning
{
StepPlan::StepPlan() {}

StepPlan::StepPlan(const StepPlan& other)
  : StepPlan(other, SharedLock(other.step_plan_mutex_))
{}

StepPlan::StepPlan(StepPlan&& other)
  : StepPlan(std::move(other), UniqueLock(other.step_plan_mutex_))
{}

StepPlan::StepPlan(const StepPlan& other, SharedLock /*other_lock*/)
  : header_(other.header_)
  , start_(other.start_)
  , goal_(other.goal_)
  , steps_(other.steps_)
{}

StepPlan::StepPlan(StepPlan&& other, UniqueLock /*other_lock*/)
  : header_(std::move(other.header_))
  , start_(std::move(other.start_))
  , goal_(std::move(other.goal_))
  , steps_(std::move(other.steps_))
{}

StepPlan& StepPlan::operator=(const StepPlan& other)
{
  if (this != &other)
  {
    UniqueLock this_lock(step_plan_mutex_, boost::defer_lock);
    SharedLock other_lock(other.step_plan_mutex_, boost::defer_lock);
    std::lock(this_lock, other_lock);

    header_ = other.header_;
    start_ = other.start_;
    goal_ = other.goal_;
    steps_ = other.steps_;
  }
  return *this;
}

StepPlan& StepPlan::operator=(StepPlan&& other)
{
  if (this != &other)
  {
    UniqueLock this_lock(step_plan_mutex_, boost::defer_lock);
    UniqueLock other_lock(other.step_plan_mutex_, boost::defer_lock);
    std::lock(this_lock, other_lock);

    header_ = std::move(other.header_);
    start_ = std::move(other.start_);
    goal_ = std::move(other.goal_);
    steps_ = std::move(other.steps_);
  }
  return *this;
}

StepPlan& StepPlan::operator=(const msgs::StepPlan& step_plan)
{
  fromMsg(step_plan);
  return *this;
}

StepPlan& StepPlan::operator+(const msgs::StepPlan& step_plan)
{
  appendStepPlan(step_plan);
  return *this;
}

StepPlan& StepPlan::operator|(const msgs::StepPlan& step_plan)
{
  updateStepPlan(step_plan);
  return *this;
}

StepPlan& StepPlan::operator+(const msgs::Step& step)
{
  insertStep(step);
  return *this;
}

StepPlan& StepPlan::operator|(const msgs::Step& step)
{
  insertStep(step);
  return *this;
}

StepPlan& StepPlan::operator-(const msgs::Step& step)
{
  removeStep(step.idx);
  return *this;
}

msgs::ErrorStatus StepPlan::fromMsg(const msgs::StepPlan& step_plan)
{
  msgs::ErrorStatus status = isConsistent(step_plan);

  // check for errors
  if (status.error != msgs::ErrorStatus::NO_ERROR)
    return status;

  UniqueLock lock(step_plan_mutex_);

  // convert step plan
  header_ = step_plan.header;
  footholdArrayMsgToL3(step_plan.start, start_);
  footholdArrayMsgToL3(step_plan.goal, goal_);
  steps_.fromMsg(step_plan.plan);

  return status;
}

msgs::ErrorStatus StepPlan::toMsg(msgs::StepPlan& step_plan) const
{
  SharedLock lock(step_plan_mutex_);

  msgs::ErrorStatus status;

  // convert step plan
  step_plan.header = header_;
  footholdArrayL3ToMsg(start_, step_plan.start);
  footholdArrayL3ToMsg(goal_, step_plan.goal);
  steps_.toMsg(step_plan.plan);

  status += isConsistent(step_plan);

  return status;
}

msgs::StepPlan StepPlan::toMsg() const
{
  msgs::StepPlan msg;
  toMsg(msg);
  return msg;
}

std::string StepPlan::toString() const
{
  std::stringstream s;
  s << std::setprecision(2) << std::fixed;
  s << std::endl << "[Step Index][Foot Index] Origin -> Target Pose" << std::endl;

  for (const StepQueue::Entry& e : steps_)
  {
    Step::ConstPtr step = e.second;

    for (const Step::FootStep::MovingDataPair& p : step->footStep().getMovingLinks())
      s << "[" << step->getStepIndex() << "] " << *p.second << std::endl;
    for (const Step::FootStep::NonMovingDataPair& p : step->footStep().getNonMovingLinks())
      s << "[" << step->getStepIndex() << "] " << *p.second << std::endl;

    s << "------------------" << std::endl;
  }

  return s.str();
}

void StepPlan::clear()
{
  UniqueLock lock(step_plan_mutex_);

  header_ = std_msgs::Header();
  start_.clear();
  goal_.clear();
  steps_.clear();
}

msgs::ErrorStatus StepPlan::insertStep(Step::Ptr step)
{
  UniqueLock lock(step_plan_mutex_);
  return _insertStep(step);
}

bool StepPlan::getStep(const msgs::StepPlan& step_plan, const StepIndex& step_idx, l3_msgs::Step& step)
{
  for (const msgs::Step& s : step_plan.plan.steps)
  {
    if (s.idx == step_idx)
    {
      step = s;
      return true;
    }
  }
  return false;
}

bool StepPlan::getStep(const StepIndex& step_idx, Step& step) const
{
  Step::Ptr step_ptr;
  if (!getStep(step_idx, step_ptr))
    return false;

  step = *step_ptr;
  return true;
}

Step::Ptr StepPlan::getStep(const StepIndex& step_idx, double dt, unsigned int closing_step_size) const
{
  if (step_idx > getLastStepIndex())
    return Step::Ptr();

  SharedLock lock(step_plan_mutex_);

  StepQueue::const_iterator itr = steps_.find(step_idx);
  if (itr == steps_.end())
    return Step::Ptr();

  Step::Ptr result = itr->second;

  // too less step data in queue
  if (closing_step_size > getLastStepIndex())
    return result;

  StepIndex max_step_idx = getLastStepIndex() - closing_step_size;

  double time = 0.0;
  while (++itr != steps_.end())
  {
    if (result->getStepIndex() >= max_step_idx)
      break;

    result = itr->second;

    // checking first time before incrementing to get a step AFTER dt
    if (time < dt)
      time += result->getStepDuration();
    else
      break;
  }

  return result;
}

const Step::Ptr StepPlan::getStepAt(unsigned int position) const
{
  SharedLock lock(step_plan_mutex_);

  if (steps_.empty())
    return Step::Ptr();
  else
  {
    StepQueue::const_iterator itr = steps_.begin();
    std::advance(itr, position);
    return itr->second;
  }
}

void StepPlan::removeStepAt(unsigned int position)
{
  Step::Ptr step = getStepAt(position);
  if (step)
    removeStep(step->getStepIndex());
}

msgs::ErrorStatus StepPlan::appendStepPlan(const StepPlan& step_plan)
{
  UniqueLock lock(step_plan_mutex_);

  // check for errors
  if (step_plan.empty())
    return ErrorStatusWarning(msgs::ErrorStatus::WARN_INVALID_STEP_PLAN, "appendStepPlan", "Got empty plan!");

  if (!empty())
  {
    if (step_plan.header_.frame_id != header_.frame_id)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "appendStepPlan", "Frame id mismatch of plans: " + header_.frame_id + " vs. " + step_plan.header_.frame_id);
  }
  // append plan
  return _appendStepPlan(step_plan);
}

msgs::ErrorStatus StepPlan::updateStepPlan(const StepPlan& step_plan)
{
  UniqueLock lock(step_plan_mutex_);

  // check for errors
  if (step_plan.empty())
    return ErrorStatusWarning(msgs::ErrorStatus::WARN_INVALID_STEP_PLAN, "updateStepPlan", "Got empty plan!");

  if (!empty())
  {
    if (step_plan.header_.frame_id != header_.frame_id)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "updateStepPlan", "Frame id mismatch of plans: " + header_.frame_id + " vs. " + step_plan.header_.frame_id);
  }

  // update plan
  return _updateStepPlan(step_plan);
}

msgs::ErrorStatus StepPlan::stitchStepPlan(const StepPlan& step_plan, const StepIndex& step_idx)
{
  UniqueLock lock(step_plan_mutex_);

  // check for errors
  if (step_plan.empty())
    return ErrorStatusWarning(msgs::ErrorStatus::WARN_INVALID_STEP_PLAN, "stitchStepPlan", "Got empty plan!");

  if (!empty())
  {
    if (step_plan.header_.frame_id != header_.frame_id)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "stitchStepPlan", "Frame id mismatch of plans: " + header_.frame_id + " vs. " + step_plan.header_.frame_id);
    else if (step_idx > 0 && step_idx > getLastStepIndex())
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "stitchStepPlan", "Can't stitch as requested step index is not in current step plan!");
    /// TODO: Check if stiching would result in consistent step plan
  }

  // stitch plan
  return _stitchStepPlan(step_plan, step_idx);
}

l3_footstep_planning_msgs::StepPlan StepPlan::transformStepPlan(const msgs::StepPlan& step_plan, const Transform& transform, const std_msgs::Header& header)
{
  StepPlan plan(step_plan);
  plan.transform(transform, header);
  return plan.toMsg();

  //  tf::Pose pose;

  //  // start pose
  //  tf::poseMsgToTF(step_plan.start.left.pose, pose);
  //  pose = transform * pose;
  //  tf::poseTFToMsg(pose, step_plan.start.left.pose);

  //  tf::poseMsgToTF(step_plan.start.right.pose, pose);
  //  pose = transform * pose;
  //  tf::poseTFToMsg(pose, step_plan.start.right.pose);

  //  // goal pose
  //  tf::poseMsgToTF(step_plan.goal.left.pose, pose);
  //  pose = transform * pose;
  //  tf::poseTFToMsg(pose, step_plan.goal.left.pose);

  //  tf::poseMsgToTF(step_plan.goal.right.pose, pose);
  //  pose = transform * pose;
  //  tf::poseTFToMsg(pose, step_plan.goal.right.pose);

  //  // entire plan
  //  for (std::vector<msgs::Step>::iterator itr = step_plan.steps.begin(); itr != step_plan.steps.end(); itr++)
  //  {
  //    msgs::StepData& step = *itr;

  //    tf::poseMsgToTF(step.foot.pose, pose);
  //    pose = transform * pose;
  //    tf::poseTFToMsg(pose, step.foot.pose);
  //  }
}

StepPlan& StepPlan::transform(const Transform& transform, const std_msgs::Header& header)
{
  UniqueLock lock(step_plan_mutex_);

  // copy header info
  if (!header.frame_id.empty())
    header_ = header;

  // transform start and goal footholds
  for (Foothold& f : start_)
  {
    f.header = header_;
    f.transform(transform);
  }

  for (Foothold& f : goal_)
  {
    f.header = header_;
    f.transform(transform);
  }

  // transform plan
  for (StepQueue::Entry& e : steps_)
    e.second->transform(transform, header);

  return *this;
}

msgs::ErrorStatus StepPlan::_insertStep(Step::Ptr step)
{
  if (step->empty())
    return ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "insertStep", "Got empty step!");

  if (steps_.empty())
  {
    if (step->footStep().hasMovingLinks())
      header_ = step->footStep().getMovingLinks().begin()->second->origin->header;
    else if (step->footStep().hasNonMovingLinks())
      header_ = step->footStep().getNonMovingLinks().begin()->second->header;
    else if (step->baseStep().hasMovingLinks())
      header_ = step->baseStep().getMovingLinks().begin()->second->origin->header;
    else if (step->baseStep().hasNonMovingLinks())
      header_ = step->baseStep().getNonMovingLinks().begin()->second->header;
    else
      return ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "insertStep", "Could not determine header!");
  }

  // check header consistency for footholds
  for (const Step::FootStep::MovingDataPair& p : step->footStep().getMovingLinks())
  {
    FootStepData::ConstPtr step_data = p.second;

    if (step_data->origin->header.frame_id != header_.frame_id)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "insertStep",
                              "Frame id mismatch! Plan: " + header_.frame_id + " vs. step origin: " + step_data->origin->header.frame_id);

    if (step_data->target->header.frame_id != header_.frame_id)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "insertStep",
                              "Frame id mismatch! Plan: " + header_.frame_id + " vs. step target: " + step_data->target->header.frame_id);
  }

  // check header consistency for flaoting bases
  for (const Step::BaseStep::MovingDataPair& p : step->baseStep().getMovingLinks())
  {
    BaseStepData::ConstPtr step_data = p.second;

    if (step_data->origin->header.frame_id != header_.frame_id)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "insertStep",
                              "Frame id mismatch! Plan: " + header_.frame_id + " vs. step origin: " + step_data->origin->header.frame_id);

    if (step_data->target->header.frame_id != header_.frame_id)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "insertStep",
                              "Frame id mismatch! Plan: " + header_.frame_id + " vs. step target: " + step_data->target->header.frame_id);
  }

  // insert step
  steps_.enqueue(step->getStepIndex(), step);

  return msgs::ErrorStatus();
}

msgs::ErrorStatus StepPlan::_appendStepPlan(const StepPlan& step_plan)
{
  Step::ConstPtr last_step;
  StepIndex step_idx = 0;

  // init header
  if (steps_.empty())
  {
    header_ = step_plan.header_;
    start_ = step_plan.start_;
  }
  else
  {
    last_step = getLastStep();
    step_idx = getLastStepIndex() + 1;
  }

  goal_ = step_plan.goal_;

  // append plan
  msgs::ErrorStatus status;
  for (const StepQueue::Entry& e : step_plan.steps_)
  {
    // copy new step and adapt step idx
    Step::Ptr step(new Step(*e.second));
    step->setStepIndex(step_idx++);

    // fill support data of first appended step with data from last step of the exisiting step plan
    if (last_step)
    {
      /// footholds: perform only when not explicitly given
      if (!step->footStep().hasNonMovingLinks())
      {
        // gather data from step data
        for (const Step::FootStep::MovingDataPair& p : last_step->footStep().getMovingLinks())
        {
          if (!step->footStep().hasMovingLink(p.first))
            step->footStep().updateNonMovingLink(p.first, p.second->target);
        }

        // gather alternatively data from support data
        for (const Step::FootStep::NonMovingDataPair& p : last_step->footStep().getNonMovingLinks())
        {
          if (!step->footStep().hasMovingLink(p.first))
            step->footStep().updateNonMovingLink(p.first, p.second);
        }
      }

      /// floating bases: perform only when not explicitly given
      if (!step->baseStep().hasNonMovingLinks())
      {
        // gather data from step data
        for (const Step::BaseStep::MovingDataPair& p : last_step->baseStep().getMovingLinks())
        {
          if (!step->baseStep().hasMovingLink(p.first))
            step->baseStep().updateNonMovingLink(p.first, p.second->target);
        }

        // gather alternatively data from support data
        for (const Step::BaseStep::NonMovingDataPair& p : last_step->baseStep().getNonMovingLinks())
        {
          if (!step->baseStep().hasMovingLink(p.first))
            step->baseStep().updateNonMovingLink(p.first, p.second);
        }
      }

      // reset pointer as we will only fill up update data of the first appended step
      last_step.reset();
    }

    status += _insertStep(step);

    if (status.error != msgs::ErrorStatus::NO_ERROR)
      break;
  }

  return status;
}

msgs::ErrorStatus StepPlan::_updateStepPlan(const StepPlan& step_plan)
{
  // init header
  if (steps_.empty())
    header_ = step_plan.header_;

  // update plan
  msgs::ErrorStatus status;
  for (const StepQueue::Entry& e : step_plan.steps_)
  {
    Step::Ptr step(new Step(*e.second));

    /// restore non-moving legs from old step if missing in updated input step
    if (!step->footStep().hasNonMovingLinks())
    {
      Step::ConstPtr old_step = getStep(step->getStepIndex());
      if (old_step)
      {
        for (const Step::FootStep::NonMovingDataPair& p : old_step->footStep().getNonMovingLinks())
        {
          if (!step->footStep().hasMovingLink(p.first))
            step->footStep().updateNonMovingLink(p.first, p.second);
        }
      }
    }

    /// restore non-moving floating bases from old step if missing in updated input step
    if (!step->baseStep().hasNonMovingLinks())
    {
      Step::ConstPtr old_step = getStep(step->getStepIndex());
      if (old_step)
      {
        for (const Step::BaseStep::NonMovingDataPair& p : old_step->baseStep().getNonMovingLinks())
        {
          if (!step->baseStep().hasMovingLink(p.first))
            step->baseStep().updateNonMovingLink(p.first, p.second);
        }
      }
    }

    // finally insert updated step
    status += _insertStep(step);

    if (status.error != msgs::ErrorStatus::NO_ERROR)
      break;
  }

  return status;
}

msgs::ErrorStatus StepPlan::_stitchStepPlan(const StepPlan& step_plan, StepIndex step_idx)
{
  if (step_plan.empty())
    return msgs::ErrorStatus();

  // simple case: current step plan is still empty
  if (steps_.empty())
    return _updateStepPlan(step_plan);

  Step::ConstPtr last_current_step;
  Step::ConstPtr first_new_step;

  // check prerequisite for stitiching
  if (step_idx == 0)
    step_idx = step_plan.getfirstStep()->getStepIndex();

  if (!hasStep(step_idx))
    return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_STEP, "stitchStepPlan",
                            "Current step plan doesn't contain step index " + boost::lexical_cast<std::string>(step_idx) + "!");

  if (!step_plan.hasStep(step_idx))
    return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_STEP, "stitchStepPlan",
                            "Input step plan doesn't contain step index " + boost::lexical_cast<std::string>(step_idx) + "!");

  // get pivot steps from current and appended step plan
  last_current_step = getStep(step_idx);
  first_new_step = step_plan.getStep(step_idx);

  if (last_current_step->empty())
    return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_STEP, "stitchStepPlan", "Last step of old step plan is empty!");
  if (first_new_step->empty())
    return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_STEP, "stitchStepPlan", "First step of new step plan is empty!");

  /// @todo: Check if ALL footholds of both ends are equal?

  // determine pivot footholds
  Foothold::ConstPtr last_current_foothold = last_current_step->getAllFootholds().front();
  Foothold::ConstPtr first_new_foothold = first_new_step->getFoothold(last_current_foothold->idx);

  if (!first_new_foothold)
    return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_STEP, "stitchStepPlan", "First step of new plan does not contain foothold id '%u'!", last_current_foothold->idx);

  // transform input plan to be relative to current plan's reference foot pose
  StepPlan step_plan_transformed = step_plan;
  step_plan_transformed.steps_.removeSteps(0, step_idx);  // remove all steps before and inclusive pivot foothold

  // determine transformation 'input plan first step' -> 'current plan last step'
  Transform transform = last_current_foothold->pose() * first_new_foothold->pose().inverse();  /// @todo: check...
  step_plan_transformed.transform(transform);

  // remove remaining tail of old step plan
  StepIndex max_step_idx = step_plan_transformed.getLastStepIndex();
  steps_.removeSteps(max_step_idx + 1);

  // update plan using the transformed step plan
  return _updateStepPlan(step_plan_transformed);
}
}  // namespace l3_footstep_planning
