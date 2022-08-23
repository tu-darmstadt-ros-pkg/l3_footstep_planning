#include <l3_global_footstep_planner/global_footstep_planner_node.h>

namespace l3_footstep_planning
{
GlobalFootstepPlannerNode::GlobalFootstepPlannerNode(ros::NodeHandle& nh)
  : FootstepPlannerNode(nh)
{}

void GlobalFootstepPlannerNode::initialize(ros::NodeHandle& nh)
{
  // init basics
  FootstepPlannerNode::initialize(nh);

  // init global footstep planner
  global_footstep_planner_.reset(new GlobalFootstepPlanner(footstep_planner_));

  // subscribe topics
  edit_step_sub_ = nh.subscribe("edit_step", 1, &GlobalFootstepPlannerNode::editStep, this);
  set_step_plan_sub_ = nh.subscribe("set_step_plan", 1, &GlobalFootstepPlannerNode::setStepPlan, this);
  stitch_step_plan_sub_ = nh.subscribe("stitch_step_plan", 1, &GlobalFootstepPlannerNode::stitchStepPlan, this);

  // start own services
  edit_step_srv_ = nh.advertiseService("edit_step", &GlobalFootstepPlannerNode::editStepService, this);
  set_step_plan_srv_ = nh.advertiseService("set_step_plan", &GlobalFootstepPlannerNode::setStepPlanService, this);
  get_step_plan_srv_ = nh.advertiseService("get_step_plan", &GlobalFootstepPlannerNode::getStepPlanService, this);
  stitch_step_plan_srv_ = nh.advertiseService("stitch_step_plan", &GlobalFootstepPlannerNode::stitchStepPlanService, this);

  // clang-format off
  // init action servers
  edit_step_as_ = SimpleActionServer<msgs::EditStepAction>::create(nh, "edit_step", true, boost::bind(&GlobalFootstepPlannerNode::editStepAction, this, boost::ref(edit_step_as_)));
  set_step_plan_as_ = SimpleActionServer<msgs::SetStepPlanAction>::create(nh, "set_step_plan", true, boost::bind(&GlobalFootstepPlannerNode::setStepPlanAction, this, boost::ref(set_step_plan_as_)));
  get_step_plan_as_ = SimpleActionServer<msgs::GetStepPlanAction>::create(nh, "get_step_plan", true, boost::bind(&GlobalFootstepPlannerNode::getStepPlanAction, this, boost::ref(get_step_plan_as_)));
  stitch_step_plan_as_ = SimpleActionServer<msgs::StitchStepPlanAction>::create(nh, "stitch_step_plan", true, boost::bind(&GlobalFootstepPlannerNode::stitchStepPlanAction, this, boost::ref(stitch_step_plan_as_)));
  // clang-format on
}

// --- Subscriber calls ---

void GlobalFootstepPlannerNode::editStep(const msgs::EditStepConstPtr& edit_step)
{
  std::vector<msgs::StepPlan> result;
  msgs::StepPlan step_plan;
  global_footstep_planner_->getStepPlan(step_plan);  // don't need to save status here
  msgs::ErrorStatus error_status = global_footstep_planner_->editStep(*edit_step, step_plan, result);

  // step_plan_pub.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(result)));
  error_status_pub_.publish(msgs::ErrorStatusConstPtr(new msgs::ErrorStatus(error_status)));
}

void GlobalFootstepPlannerNode::setStepPlan(const msgs::StepPlanConstPtr& step_plan)
{
  msgs::ErrorStatus error_status = global_footstep_planner_->setStepPlan(*step_plan);
  msgs::StepPlan result;
  error_status += global_footstep_planner_->getStepPlan(result);

  error_status_pub_.publish(msgs::ErrorStatusConstPtr(new msgs::ErrorStatus(error_status)));
}

void GlobalFootstepPlannerNode::stitchStepPlan(const std::vector<msgs::StepPlan>& step_plans)
{
  msgs::StepPlan result;
  msgs::ErrorStatus error_status = global_footstep_planner_->stitchStepPlan(step_plans, result);

  step_plan_pub_.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(result)));
  step_plan_vis_pub_.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(result)));
  error_status_pub_.publish(msgs::ErrorStatusConstPtr(new msgs::ErrorStatus(error_status)));
}

void GlobalFootstepPlannerNode::stitchStepPlan(const msgs::StepPlanConstPtr& step_plan)
{
  std::vector<msgs::StepPlan> step_plans;

  msgs::StepPlan current_step_plan;
  global_footstep_planner_->getStepPlan(current_step_plan);
  step_plans.push_back(current_step_plan);
  step_plans.push_back(*step_plan);

  stitchStepPlan(step_plans);
}

// --- Service calls ---

bool GlobalFootstepPlannerNode::editStepService(msgs::EditStepService::Request& req, msgs::EditStepService::Response& resp)
{
  resp.status = global_footstep_planner_->editStep(req.edit_step, req.step_plan, resp.step_plans);

  // temp_step_plan_pub.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(resp.step_plans)));
  error_status_pub_.publish(msgs::ErrorStatusConstPtr(new msgs::ErrorStatus(resp.status)));

  return true;  // return always true so the message is returned
}

bool GlobalFootstepPlannerNode::setStepPlanService(msgs::SetStepPlanService::Request& req, msgs::SetStepPlanService::Response& resp)
{
  resp.status = global_footstep_planner_->setStepPlan(req.step_plan);
  resp.status += global_footstep_planner_->getStepPlan(resp.step_plan);

  error_status_pub_.publish(msgs::ErrorStatusConstPtr(new msgs::ErrorStatus(resp.status)));

  return true;  // return always true so the message is returned
}

bool GlobalFootstepPlannerNode::getStepPlanService(msgs::GetStepPlanService::Request& req, msgs::GetStepPlanService::Response& resp)
{
  resp.status = global_footstep_planner_->getStepPlan(resp.step_plan);
  return true;  // return always true so the message is returned
}

bool GlobalFootstepPlannerNode::stitchStepPlanService(msgs::StitchStepPlanService::Request& req, msgs::StitchStepPlanService::Response& resp)
{
  resp.status = global_footstep_planner_->stitchStepPlan(req.step_plans, resp.step_plan);

  temp_step_plan_pub_.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(resp.step_plan)));
  step_plan_vis_pub_.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(resp.step_plan)));
  error_status_pub_.publish(msgs::ErrorStatusConstPtr(new msgs::ErrorStatus(resp.status)));

  return true;  // return always true so the message is returned
}

//--- action server calls ---

void GlobalFootstepPlannerNode::editStepAction(SimpleActionServer<msgs::EditStepAction>::Ptr& as)
{
  const msgs::EditStepGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::EditStepResult result;
  result.status = global_footstep_planner_->editStep(goal->edit_step, goal->step_plan, result.step_plans);

  actionServerFinished(*as, result);
  // if (result.status.error == msgs::ErrorStatus::NO_ERROR)
  //   temp_step_plan_pub.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(result.step_plan)));
  error_status_pub_.publish(msgs::ErrorStatusConstPtr(new msgs::ErrorStatus(result.status)));
}

void GlobalFootstepPlannerNode::setStepPlanAction(SimpleActionServer<msgs::SetStepPlanAction>::Ptr& as)
{
  const msgs::SetStepPlanGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::SetStepPlanResult result;
  result.status = global_footstep_planner_->setStepPlan(goal->step_plan);
  result.status += global_footstep_planner_->getStepPlan(result.step_plan);

  actionServerFinished(*as, result);
  error_status_pub_.publish(msgs::ErrorStatusConstPtr(new msgs::ErrorStatus(result.status)));
}

void GlobalFootstepPlannerNode::getStepPlanAction(SimpleActionServer<msgs::GetStepPlanAction>::Ptr& as)
{
  as->acceptNewGoal();

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::GetStepPlanResult result;
  result.status = global_footstep_planner_->getStepPlan(result.step_plan);
  as->setSucceeded(result, toString(result.status));
}

void GlobalFootstepPlannerNode::stitchStepPlanAction(SimpleActionServer<msgs::StitchStepPlanAction>::Ptr& as)
{
  const msgs::StitchStepPlanGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::StitchStepPlanResult result;
  result.status = global_footstep_planner_->stitchStepPlan(goal->step_plans, result.step_plan);

  actionServerFinished(*as, result);
  temp_step_plan_pub_.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(result.step_plan)));
  step_plan_vis_pub_.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(result.step_plan)));
  error_status_pub_.publish(msgs::ErrorStatusConstPtr(new msgs::ErrorStatus(result.status)));
}
}  // namespace l3_footstep_planning

int main(int argc, char** argv)
{
  ros::init(argc, argv, "l3_global_footstep_planner");

  ros::NodeHandle nh;

  // ensure that node's services are set up in proper namespace
  if (nh.getNamespace().size() <= 1)
    nh = ros::NodeHandle("~");

  l3_footstep_planning::GlobalFootstepPlannerNode globalPlannerNode(nh);
  globalPlannerNode.initialize(nh);

  ros::spin();

  return 0;
}
