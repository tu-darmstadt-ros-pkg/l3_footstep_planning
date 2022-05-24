#include <l3_footstep_planner/footstep_planner_node.h>

#include <vigir_pluginlib/plugin_manager.h>

#include <l3_plugins/robot_model.h>

#include <l3_libs/conversions/l3_msg_conversions.h>

#include <l3_footstep_planning_libs/helper.h>

#include <l3_footstep_planning_plugins/base/use_mask_generator_plugin.h>
#include <l3_footstep_planning_plugins/base/state_generator_plugin.h>
#include <l3_footstep_planning_plugins/base/reachability_plugin.h>
#include <l3_footstep_planning_plugins/base/step_cost_estimator_plugin.h>
#include <l3_footstep_planning_plugins/base/heuristic_plugin.h>
#include <l3_footstep_planning_plugins/base/post_process_plugin.h>
#include <l3_footstep_planning_plugins/base/world_model_plugin.h>
#include <l3_footstep_planning_plugins/base/terrain_model_plugin.h>
#include <l3_footstep_planning_plugins/base/hfs_heuristic_plugin.h>
#include <l3_footstep_planning_plugins/aggregator/post_processor.h>

#include <l3_footstep_planning_tools/foot_pose_transformer.h>
#include <l3_footstep_planning_tools/feet_pose_generator.h>

namespace l3_footstep_planning
{
FootstepPlannerNode::FootstepPlannerNode(ros::NodeHandle& nh)
{
  // init parameter and plugin manager
  vigir_generic_params::ParameterManager::initialize(nh);
  vigir_pluginlib::PluginManager::initialize(nh);

  // initialize robot model
  RobotModel::initialize(nh);
}

void FootstepPlannerNode::initPlugins(ros::NodeHandle& /*nh*/)
{
  vigir_pluginlib::PluginManager::addPluginClassLoader<UseMaskGeneratorPlugin>("l3_footstep_planning_plugins", "l3_footstep_planning::UseMaskGeneratorPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<StateGeneratorPlugin>("l3_footstep_planning_plugins", "l3_footstep_planning::StateGeneratorPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<ReachabilityPlugin>("l3_footstep_planning_plugins", "l3_footstep_planning::ReachabilityPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<StepCostEstimatorPlugin>("l3_footstep_planning_plugins", "l3_footstep_planning::StepCostEstimatorPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<HeuristicPlugin>("l3_footstep_planning_plugins", "l3_footstep_planning::HeuristicPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<PostProcessPlugin>("l3_footstep_planning_plugins", "l3_footstep_planning::PostProcessPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<WorldModelPlugin>("l3_footstep_planning_plugins", "l3_footstep_planning::WorldModelPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<TerrainModelPlugin>("l3_footstep_planning_plugins", "l3_footstep_planning::TerrainModelPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<HFSHeuristicPlugin>("l3_footstep_planning_plugins", "l3_footstep_planning::HFSHeuristicPlugin");
}

void FootstepPlannerNode::initialize(ros::NodeHandle& nh)
{
  initPlugins(nh);

  // init planner
  footstep_planner.reset(new FootstepPlanner(nh));

  // subscribe topics
  set_active_parameter_set_sub = nh.subscribe<std_msgs::String>("set_active_parameter_set", 1, &FootstepPlannerNode::setParams, this);
  step_plan_request_sub = nh.subscribe("step_plan_request", 1, &FootstepPlannerNode::stepPlanRequest, this);
  goal_pose_sub = nh.subscribe("/goal", 1, &FootstepPlannerNode::goalPoseCallback, this);

  // publish topics
  step_plan_pub = nh.advertise<msgs::StepPlan>("step_plan", 1);
  step_plan_request_vis_pub = nh.advertise<msgs::StepPlanRequest>("vis/step_plan_request", 1);
  step_plan_vis_pub = nh.advertise<msgs::StepPlan>("vis/step_plan", 1);
  error_status_pub = nh.advertise<msgs::ErrorStatus>("error_status", 1);
  temp_step_plan_pub = nh.advertise<msgs::StepPlan>("temp_step_plan", 1);
  feedback_pub = nh.advertise<msgs::PlanningFeedback>("planning_feedback", 1);

  // start service clients
  generate_feet_pose_client = nh.serviceClient<msgs::GenerateFeetPoseService>("generate_feet_pose");

  // start own services
  step_plan_request_srv = nh.advertiseService("step_plan_request", &FootstepPlannerNode::stepPlanRequestService, this);
  update_foot_srv = nh.advertiseService("update_foot", &FootstepPlannerNode::updateFootService, this);
  update_feet_srv = nh.advertiseService("update_feet", &FootstepPlannerNode::updateFeetService, this);
  update_step_plan_srv = nh.advertiseService("update_step_plan", &FootstepPlannerNode::updateStepPlanService, this);

  // clang-format off
  // init action servers
  step_plan_request_as = SimpleActionServer<msgs::StepPlanRequestAction>::create(nh, "step_plan_request", true,
                                                                                 boost::bind(&FootstepPlannerNode::stepPlanRequestAction, this, boost::ref(step_plan_request_as)),
                                                                                 boost::bind(&FootstepPlannerNode::stepPlanRequestPreempt, this, boost::ref(step_plan_request_as)));
  update_foot_as = SimpleActionServer<msgs::UpdateFootAction>::create(nh, "update_foot", true, boost::bind(&FootstepPlannerNode::updateFootAction, this, boost::ref(update_foot_as)));
  update_feet_as = SimpleActionServer<msgs::UpdateFeetAction>::create(nh, "update_feet", true, boost::bind(&FootstepPlannerNode::updateFeetAction, this, boost::ref(update_feet_as)));
  update_step_plan_as = SimpleActionServer<msgs::UpdateStepPlanAction>::create(nh, "update_step_plan", true, boost::bind(&FootstepPlannerNode::updateStepPlanAction, this, boost::ref(update_step_plan_as)));
  // clang-format on
}

void FootstepPlannerNode::postProcessRequestVis(msgs::StepPlanRequest& req) const
{
  /// post process start state

  FootPoseTransformer::transformToPlannerFrame(req.start_footholds);
  FootholdArray footholds;
  footholdArrayMsgToL3(req.start_footholds, footholds);

  // post process footholds
  State s;
  for (Foothold& f : footholds)
  {
    PostProcessor::instance().postProcess(f);
    s.updateFoothold(StateSpaceManager::addFoothold(f));
  }

  ///@todo Implement FloatingBasePostTransformer
  // post process floating bases
  if (s.hasFloatingBases())
  {
    for (const FloatingBase::ConstPtr& fb : s.getFloatingBases())
    {
      FloatingBase base(*fb);
      PostProcessor::instance().postProcess(base);
      s.updateFloatingBase(StateSpaceManager::addFloatingBase(base));
    }
  }
  else
  {
    FloatingBaseArray floating_bases;
    floatingBaseArrayMsgToL3(req.start_floating_bases, floating_bases);
    for (FloatingBase& fb : floating_bases)
    {
      PostProcessor::instance().postProcess(fb);
      s.updateFloatingBase(StateSpaceManager::addFloatingBase(fb));
    }
  }

  // post process state
  PostProcessor::instance().postProcess(s);

  // save result in request
  footholdArrayL3ToMsg(s.getFootholds(), req.start_footholds);
  FootPoseTransformer::transformToRobotFrame(req.start_footholds);

  floatingBaseArrayL3ToMsg(s.getFloatingBases(), req.start_floating_bases);

  /// post process goal state

  FootPoseTransformer::transformToPlannerFrame(req.goal_footholds);
  footholdArrayMsgToL3(req.goal_footholds, footholds);

  // post process footholds
  s = State();
  for (Foothold& f : footholds)
  {
    PostProcessor::instance().postProcess(f);
    s.updateFoothold(StateSpaceManager::addFoothold(f));
  }

  ///@todo Implement FloatingBasePostTransformer
  // post process floating bases
  if (s.hasFloatingBases())
  {
    for (const FloatingBase::ConstPtr& fb : s.getFloatingBases())
    {
      FloatingBase base(*fb);
      PostProcessor::instance().postProcess(base);
      s.updateFloatingBase(StateSpaceManager::addFloatingBase(base));
    }
  }
  else
  {
    FloatingBaseArray floating_bases;
    floatingBaseArrayMsgToL3(req.goal_floating_bases, floating_bases);
    for (FloatingBase& fb : floating_bases)
    {
      PostProcessor::instance().postProcess(fb);
      s.updateFloatingBase(StateSpaceManager::addFloatingBase(fb));
    }
  }

  // post process state
  PostProcessor::instance().postProcess(s);

  // save result in request
  footholdArrayL3ToMsg(s.getFootholds(), req.goal_footholds);
  FootPoseTransformer::transformToRobotFrame(req.goal_footholds);

  floatingBaseArrayL3ToMsg(s.getFloatingBases(), req.goal_floating_bases);
}

// --- Callbacks ---

void FootstepPlannerNode::planningResultCallback(const msgs::StepPlanRequestService::Response& resp)
{
  // check result
  if (hasError(resp.status))
  {
    ROS_ERROR("[FootstepPlannerNode] Error while planning steps:\n%s", toString(resp.status).c_str());
    return;
  }
  else if (hasWarning(resp.status))
    ROS_WARN("[FootstepPlannerNode] Warning occured:\n%s", toString(resp.status).c_str());

  // publish and visualize plan
  step_plan_pub.publish(resp.step_plan);
  temp_step_plan_pub.publish(resp.step_plan);
  step_plan_vis_pub.publish(resp.step_plan);
  error_status_pub.publish(resp.status);
}

void FootstepPlannerNode::planningResultActionCallback(const msgs::StepPlanRequestService::Response& resp, SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  boost::recursive_mutex::scoped_lock lock(step_plan_request_as_mutex);

  // finish action server
  msgs::StepPlanRequestResult result;

  result.step_plan = resp.step_plan;
  result.status = resp.status;
  result.statistics = resp.statistics;

  actionServerFinished(*as, result);

  // check result
  if (hasError(resp.status))
  {
    ROS_ERROR("[FootstepPlannerNode] Error while planning steps:\n%s", toString(resp.status).c_str());
    return;
  }
  else if (hasWarning(resp.status))
    ROS_WARN("[FootstepPlannerNode] Warning occured:\n%s", toString(resp.status).c_str());

  // publish and visualize plan
  temp_step_plan_pub.publish(resp.step_plan);
  step_plan_vis_pub.publish(resp.step_plan);
  error_status_pub.publish(resp.status);
}

void FootstepPlannerNode::planningFeedbackCallback(const msgs::PlanningFeedback& feedback) { feedback_pub.publish(feedback); }

void FootstepPlannerNode::planningFeedbackActionCallback(const msgs::PlanningFeedback& feedback, SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  planningFeedbackCallback(feedback);

  msgs::StepPlanRequestFeedback fb;
  fb.feedback = feedback;
  as->publishFeedback(fb);
}

void FootstepPlannerNode::planningPreemptionActionCallback(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  boost::recursive_mutex::scoped_lock lock(step_plan_request_as_mutex);

  if (as->isActive())
    as->setPreempted();
}

// --- Subscriber calls ---

void FootstepPlannerNode::setParams(const std_msgs::StringConstPtr& params_name)
{
  vigir_generic_params::ParameterSet params;

  if (!ParameterManager::getParameterSet(params_name->data, params))
    ROS_ERROR("[FootstepPlannerNode] setParams: Unknown parameter set '%s'!", params_name->data.c_str());
  else if (!footstep_planner->setParams(params))
    ROS_ERROR("[FootstepPlannerNode] setParams: Couldn't set parameter set '%s'!", params_name->data.c_str());
  else
    ParameterManager::setActive(params_name->data);
}

void FootstepPlannerNode::stepPlanRequest(const msgs::StepPlanRequestConstPtr& plan_request)
{
  msgs::ErrorStatus status;
  msgs::StepPlanRequestService::Request step_plan_request;
  step_plan_request.plan_request = *plan_request;

  // generate start feet pose if needed
  if (step_plan_request.plan_request.start_footholds.empty())
  {
    status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlannerNode",
                                 "stepPlanRequest: No valid frame_id was given as start pose. Try to use current robot pose as start.");
    status += determineStartFootholds(step_plan_request.plan_request.start_footholds, generate_feet_pose_client, step_plan_request.plan_request.header);

    if (hasError(status))
    {
      ROS_WARN("[FootstepPlannerNode] Can't obtain start feet pose:\n%s", toString(status).c_str());
      return;
    }
    else if (hasWarning(status))
      ROS_WARN("[FootstepPlannerNode] Warning occured while obtaining start feet pose:\n%s", toString(status).c_str());
  }

  // clang-format off
  // start planning
  status = footstep_planner->stepPlanRequest(step_plan_request,
                                             boost::bind(&FootstepPlannerNode::planningResultCallback, this, _1),
                                             boost::bind(&FootstepPlannerNode::planningFeedbackCallback, this, _1));
  // clang-format on

  // visualize request
  msgs::StepPlanRequest req_vis = step_plan_request.plan_request;
  postProcessRequestVis(req_vis);
  step_plan_request_vis_pub.publish(req_vis);

  if (!isOk(status))
    ROS_INFO("[FootstepPlannerNode] stepPlanRequest:\n%s", toString(status).c_str());
}

void FootstepPlannerNode::goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose)
{
  // get start feet pose
  msgs::FootholdArray start_footholds;
  msgs::ErrorStatus status = determineStartFootholds(start_footholds, generate_feet_pose_client, goal_pose->header);

  if (hasError(status))
  {
    ROS_WARN("[FootstepPlannerNode] Can't obtain start feet pose:\n%s", toString(status).c_str());
    return;
  }
  else if (hasWarning(status))
    ROS_WARN("[FootstepPlannerNode] Warning occured while obtaining start feet pose:\n%s", toString(status).c_str());

  // get goal feet pose
  msgs::FootholdArray goal_footholds;
  status = determineGoalFootholds(goal_footholds, generate_feet_pose_client, *goal_pose);

  if (hasError(status))
  {
    ROS_WARN("[FootstepPlannerNode] Can't obtain goal feet pose:\n%s", toString(status).c_str());
    return;
  }
  else if (hasWarning(status))
    ROS_WARN("[FootstepPlannerNode] Warning occured while obtaining goal feet pose:\n%s", toString(status).c_str());

  footstep_planner->updateFeet(goal_footholds, msgs::UpdateMode::UPDATE_MODE_MOVE_TO_VALID);

  // request step plan
  msgs::StepPlanRequestService::Request step_plan_request;
  step_plan_request.plan_request.header = goal_pose->header;
  step_plan_request.plan_request.start_footholds = start_footholds;
  step_plan_request.plan_request.goal_footholds = goal_footholds;
  step_plan_request.plan_request.start_foot_idx = msgs::StepPlanRequest::AUTO_START_FOOT_IDX;
  step_plan_request.plan_request.start_step_idx = 0;
  step_plan_request.plan_request.planning_mode = WorldModel::instance().isTerrainModelAvailable() ? static_cast<uint8_t>(msgs::StepPlanRequest::PLANNING_MODE_3D) :
                                                                                                    static_cast<uint8_t>(msgs::StepPlanRequest::PLANNING_MODE_2D);
  step_plan_request.plan_request.max_planning_time = 0.0;
  step_plan_request.plan_request.parameter_set_name.data = std::string();

  // start planning
  status = footstep_planner->stepPlanRequest(step_plan_request, boost::bind(&FootstepPlannerNode::planningResultCallback, this, _1),
                                             boost::bind(&FootstepPlannerNode::planningFeedbackCallback, this, _1));

  // visualize request

  msgs::StepPlanRequest req_vis = step_plan_request.plan_request;
  postProcessRequestVis(req_vis);
  step_plan_request_vis_pub.publish(req_vis);

  if (!isOk(status))
    ROS_INFO("[FootstepPlannerNode] goalPoseCallback:\n%s", toString(status).c_str());
}

// --- service calls ---

bool FootstepPlannerNode::stepPlanRequestService(msgs::StepPlanRequestService::Request& req, msgs::StepPlanRequestService::Response& resp)
{
  // generate start feet pose if needed
  if (req.plan_request.start_footholds.empty())
  {
    resp.status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlannerNode",
                                      "stepPlanRequestService: No valid frame_id was given as start pose. Try to use current robot pose as start.");
    resp.status += determineStartFootholds(req.plan_request.start_footholds, generate_feet_pose_client, req.plan_request.header);
  }

  // start planning
  if (!footstep_planner->stepPlanRequestService(req, resp))
    resp.status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlannerNode", "stepPlanRequestService: Can't call footstep planner service!");

  // visualize request
  msgs::StepPlanRequest req_vis = req.plan_request;
  postProcessRequestVis(req_vis);
  step_plan_request_vis_pub.publish(req_vis);

  temp_step_plan_pub.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(resp.step_plan)));
  step_plan_vis_pub.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(resp.step_plan)));
  error_status_pub.publish(msgs::ErrorStatusConstPtr(new msgs::ErrorStatus(resp.status)));

  return true;  // return always true so the message is returned
}

bool FootstepPlannerNode::updateFootService(msgs::UpdateFootService::Request& req, msgs::UpdateFootService::Response& resp)
{
  resp.foot = req.foot;
  resp.status = footstep_planner->updateFoot(resp.foot, req.update_mode.mode);
  return true;  // return always true so the message is returned
}

bool FootstepPlannerNode::updateFeetService(msgs::UpdateFeetService::Request& req, msgs::UpdateFeetService::Response& resp)
{
  resp.feet = req.feet;
  resp.status = footstep_planner->updateFeet(resp.feet, req.update_mode.mode);
  return true;  // return always true so the message is returned
}

bool FootstepPlannerNode::updateStepPlanService(msgs::UpdateStepPlanService::Request& req, msgs::UpdateStepPlanService::Response& resp)
{
  resp.step_plan = req.step_plan;
  resp.status = footstep_planner->updateStepPlan(resp.step_plan, req.update_mode.mode, req.parameter_set_name.data);
  step_plan_vis_pub.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(resp.step_plan)));
  return true;  // return always true so the message is returned
}

//--- action server calls ---

void FootstepPlannerNode::stepPlanRequestAction(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  // preempt any previous goal if active due to given callback
  footstep_planner->preemptPlanning();

  boost::recursive_mutex::scoped_lock lock(step_plan_request_as_mutex);

  // accept new goal
  const msgs::StepPlanRequestGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::ErrorStatus status;
  msgs::StepPlanRequestService::Request step_plan_request;
  step_plan_request.plan_request = goal->plan_request;

  // generate start feet pose if needed
  if (step_plan_request.plan_request.start_footholds.empty())
  {
    status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlannerNode",
                                 "stepPlanRequestAction: No valid frame_id was given as start pose. Try to use current robot pose as start.");
    status += determineStartFootholds(step_plan_request.plan_request.start_footholds, generate_feet_pose_client, step_plan_request.plan_request.header);
  }

  // clang-format off
  // start planning
  status = footstep_planner->stepPlanRequest(step_plan_request,
                                             boost::bind(&FootstepPlannerNode::planningResultActionCallback, this, _1, boost::ref(as)),
                                             boost::bind(&FootstepPlannerNode::planningFeedbackActionCallback, this, _1, boost::ref(as)),
                                             boost::bind(&FootstepPlannerNode::planningPreemptionActionCallback, this, boost::ref(as)));
  // clang-format on

  // visualize request
  msgs::StepPlanRequest req_vis = step_plan_request.plan_request;
  postProcessRequestVis(req_vis);
  step_plan_request_vis_pub.publish(req_vis);

  if (!isOk(status))
    ROS_INFO("[FootstepPlannerNode] stepPlanRequest:\n%s", toString(status).c_str());
}

void FootstepPlannerNode::stepPlanRequestPreempt(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  boost::recursive_mutex::scoped_lock lock(step_plan_request_as_mutex);

  if (as->isActive())
  {
    footstep_planner->preemptPlanning();
    as->setPreempted();
  }
}

void FootstepPlannerNode::updateFootAction(SimpleActionServer<msgs::UpdateFootAction>::Ptr& as)
{
  const msgs::UpdateFootGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::UpdateFootResult result;
  result.foot = goal->foot;
  result.status = footstep_planner->updateFoot(result.foot, goal->update_mode.mode);

  actionServerFinished(*as, result);
}

void FootstepPlannerNode::updateFeetAction(SimpleActionServer<msgs::UpdateFeetAction>::Ptr& as)
{
  const msgs::UpdateFeetGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::UpdateFeetResult result;
  result.feet = goal->feet;
  result.status = footstep_planner->updateFeet(result.feet, goal->update_mode.mode);

  actionServerFinished(*as, result);
}

void FootstepPlannerNode::updateStepPlanAction(SimpleActionServer<msgs::UpdateStepPlanAction>::Ptr& as)
{
  const msgs::UpdateStepPlanGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::UpdateStepPlanResult result;

  result.step_plan = goal->step_plan;
  result.status = footstep_planner->updateStepPlan(result.step_plan, goal->update_mode.mode, goal->parameter_set_name.data);
  step_plan_vis_pub.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(result.step_plan)));

  actionServerFinished(*as, result);
}
}  // namespace l3_footstep_planning

int main(int argc, char** argv)
{
  ros::init(argc, argv, "l3_footstep_planner");

  ros::NodeHandle nh;

  // ensure that node's services are set up in proper namespace
  if (nh.getNamespace().size() <= 1)
    nh = ros::NodeHandle("~");

  // init footstep planner
  l3_footstep_planning::FootstepPlannerNode footstep_planner_node(nh);
  footstep_planner_node.initialize(nh);

  ros::spin();

  return 0;
}
