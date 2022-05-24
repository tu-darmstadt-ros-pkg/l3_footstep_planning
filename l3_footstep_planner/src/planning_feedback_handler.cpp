#include <l3_footstep_planner/planning_feedback_handler.h>

#include <l3_libs/conversions/l3_msg_conversions.h>

#include <l3_footstep_planning_tools/foot_pose_transformer.h>

namespace l3_footstep_planning
{
PlanningFeedbackHandler::PlanningFeedbackHandler(ros::NodeHandle& nh)
{
  feedback_timer_ = nh.createTimer(ros::Rate(1.0), &PlanningFeedbackHandler::publishFeedback, this, false, false);
}

void PlanningFeedbackHandler::setParameters(const EnvironmentParameters& params)
{
  UniqueLock lock(data_mutex_);
  forward_search_ = params.forward_search;
}

void PlanningFeedbackHandler::setFrameId(const std::string& frame_id)
{
  UniqueLock lock(data_mutex_);
  this->frame_id_ = frame_id;
}

void PlanningFeedbackHandler::setFeedbackCB(boost::function<void(msgs::PlanningFeedback)>& feedback_cb)
{
  UniqueLock lock(data_mutex_);
  feedback_cb_ = feedback_cb;
}

void PlanningFeedbackHandler::reset()
{
  clear();
  stop();
}

void PlanningFeedbackHandler::clear()
{
  UniqueLock lock(data_mutex_);
  visited_steps_.clear();
  last_visited_state_.reset();
}

void PlanningFeedbackHandler::start(double rate)
{
  UniqueLock lock(timer_mutex_);
  feedback_timer_.setPeriod(ros::Duration(1.0 / rate));
  feedback_timer_.start();
}

void PlanningFeedbackHandler::stop()
{
  UniqueLock lock(timer_mutex_);
  feedback_timer_.stop();
}

void PlanningFeedbackHandler::stateExpanded(PlanningState::ConstPtr state)
{
  // no upgrade lock used to prevent overhead and deadlocks
  if (!feedback_cb_.empty())
  {
    UniqueLock lock(data_mutex_);
    last_visited_state_ = state;
  }
}

void PlanningFeedbackHandler::stateVisited(PlanningState::ConstPtr state)
{
  // no upgrade lock used to prevent overhead and deadlocks
  if (!feedback_cb_.empty())
  {
    UniqueLock lock(data_mutex_);
    visited_steps_.push_back(state->getStep());
  }
}

void PlanningFeedbackHandler::publishFeedback(const ros::TimerEvent& /*event*/)
{
  // no upgrade lock used to prevent overhead and deadlocks
  if (feedback_cb_.empty())
    return;

  UniqueLock lock(data_mutex_);

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = frame_id_;

  // send recent visited states
  msgs::PlanningFeedback feedback;
  feedback.header = header;

  // add visited steps
  for (Step::ConstPtr s : visited_steps_)
  {
    for (Step::StepDataPair p : s->getStepDataMap())
    {
      msgs::StepData step_msg = p.second->toMsg();
      step_msg.origin.header = step_msg.target.header = header;
      FootPoseTransformer::transformToRobotFrame(step_msg);
      feedback.visited_steps.push_back(step_msg);
    }
    for (Step::BaseStepDataPair p : s->getMovingFloatingBaseMap())
    {
      ///@todo Implement BasePoseTransformer
      msgs::BaseStepData base_step_msg = p.second->toMsg();
      base_step_msg.origin.header = base_step_msg.target.header = header;
      feedback.visited_bases.push_back(base_step_msg);
    }
  }

  // add last visited state
  if (last_visited_state_)
  {
    footholdArrayL3ToMsg(last_visited_state_->getState()->getFootholds(), feedback.last_visited_state);
    for (msgs::Foothold& f : feedback.last_visited_state)
      f.header = feedback.header;
    FootPoseTransformer::transformToRobotFrame(feedback.last_visited_state);

    if (last_visited_state_->getStep())
    {
      stepL3ToMsg(*last_visited_state_->getStep(), feedback.last_visited_step);
      for (msgs::StepData& s : feedback.last_visited_step.step_data)
      {
        s.origin.header = header;
        s.target.header = header;
      }
      FootPoseTransformer::transformToRobotFrame(feedback.last_visited_step);
    }
  }  

  feedback.current_step_plan.header = header;

  /// @todo: requires transition db
  //      while (state)
  //      {
  //        /// @TODO: Header should be set when state is created
  //        msgs::StepData step_msg = state->getSteps().begin()->second.toMsg(); /// @TODO
  //        step_msg.foothold.header = feedback.header;
  //        feedback.current_step_plan.steps.push_back(step_msg);

  //        if (!state->getPredState())
  //          break;
  //        state = state->getPredState();
  //      }

  // transform step plan
  FootPoseTransformer::transformToRobotFrame(feedback.current_step_plan.steps);

  feedback.forward_search = forward_search_;

  // publish feedback
  feedback_cb_(feedback);

  visited_steps_.clear();
  last_visited_state_.reset();
}
}  // namespace l3_footstep_planning
