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

  // init feet pose generator
  feet_pose_generator_ = makeShared<FeetPoseGeneratorClient>(nh);

  // init planner
  footstep_planner_ = makeShared<FootstepPlanner>(nh);

  // subscribe topics
  set_active_parameter_set_sub_ = nh.subscribe<std_msgs::String>("set_active_parameter_set", 1, &FootstepPlannerNode::setParams, this);
  step_plan_request_sub_ = nh.subscribe("step_plan_request", 1, &FootstepPlannerNode::stepPlanRequest, this);
  goal_pose_sub_ = nh.subscribe("/goal", 1, &FootstepPlannerNode::goalPoseCallback, this);

  // publish topics
  step_plan_pub_ = nh.advertise<msgs::StepPlan>("step_plan", 1, true);
  step_plan_request_vis_pub_ = nh.advertise<msgs::StepPlanRequest>("vis/step_plan_request", 1, true);
  step_plan_vis_pub_ = nh.advertise<msgs::StepPlan>("vis/step_plan", 1, true);
  error_status_pub_ = nh.advertise<msgs::ErrorStatus>("error_status", 1, true);
  temp_step_plan_pub_ = nh.advertise<msgs::StepPlan>("temp_step_plan", 1, true);
  feedback_pub_ = nh.advertise<msgs::PlanningFeedback>("planning_feedback", 1, true);

  // start own services
  step_plan_request_srv_ = nh.advertiseService("step_plan_request", &FootstepPlannerNode::stepPlanRequestService, this);
  update_foot_srv_ = nh.advertiseService("update_foot", &FootstepPlannerNode::updateFootService, this);
  update_feet_srv_ = nh.advertiseService("update_feet", &FootstepPlannerNode::updateFeetService, this);
  update_step_plan_srv_ = nh.advertiseService("update_step_plan", &FootstepPlannerNode::updateStepPlanService, this);

  // clang-format off
  // init action servers
  step_plan_request_as_ = SimpleActionServer<msgs::StepPlanRequestAction>::create(nh, "step_plan_request", true,
                                                                                  boost::bind(&FootstepPlannerNode::stepPlanRequestAction, this, boost::ref(step_plan_request_as_)),
                                                                                  boost::bind(&FootstepPlannerNode::stepPlanRequestPreempt, this, boost::ref(step_plan_request_as_)));
  update_foot_as_ = SimpleActionServer<msgs::UpdateFootAction>::create(nh, "update_foot", true, boost::bind(&FootstepPlannerNode::updateFootAction, this, boost::ref(update_foot_as_)));
  update_feet_as_ = SimpleActionServer<msgs::UpdateFeetAction>::create(nh, "update_feet", true, boost::bind(&FootstepPlannerNode::updateFeetAction, this, boost::ref(update_feet_as_)));
  update_step_plan_as_ = SimpleActionServer<msgs::UpdateStepPlanAction>::create(nh, "update_step_plan", true, boost::bind(&FootstepPlannerNode::updateStepPlanAction, this, boost::ref(update_step_plan_as_)));
  // clang-format on
}

/// --- Callbacks ---

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
  step_plan_pub_.publish(resp.step_plan);
  temp_step_plan_pub_.publish(resp.step_plan);
  step_plan_vis_pub_.publish(resp.step_plan);
  error_status_pub_.publish(resp.status);
}

void FootstepPlannerNode::planningResultActionCallback(const msgs::StepPlanRequestService::Response& resp, SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  boost::recursive_mutex::scoped_lock lock(step_plan_request_as_mutex_);

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
  temp_step_plan_pub_.publish(resp.step_plan);
  step_plan_vis_pub_.publish(resp.step_plan);
  error_status_pub_.publish(resp.status);
}

void FootstepPlannerNode::planningFeedbackCallback(const msgs::PlanningFeedback& feedback) { feedback_pub_.publish(feedback); }

void FootstepPlannerNode::planningFeedbackActionCallback(const msgs::PlanningFeedback& feedback, SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  planningFeedbackCallback(feedback);

  msgs::StepPlanRequestFeedback fb;
  fb.feedback = feedback;
  as->publishFeedback(fb);
}

void FootstepPlannerNode::planningPreemptionActionCallback(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  boost::recursive_mutex::scoped_lock lock(step_plan_request_as_mutex_);

  if (as->isActive())
    as->setPreempted();
}

/// --- Subscriber calls ---

void FootstepPlannerNode::setParams(const std_msgs::StringConstPtr& params_name)
{
  ParameterSet params;

  if (!ParameterManager::getParameterSet(params_name->data, params))
    ROS_ERROR("[FootstepPlannerNode] setParams: Unknown parameter set '%s'!", params_name->data.c_str());
  else if (!footstep_planner_->setParams(params))
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
    status += feet_pose_generator_->getStartFootholds(step_plan_request.plan_request.start_footholds, step_plan_request.plan_request.header.frame_id);

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
  status = footstep_planner_->stepPlanRequest(step_plan_request,
                                             boost::bind(&FootstepPlannerNode::planningResultCallback, this, _1),
                                             boost::bind(&FootstepPlannerNode::planningFeedbackCallback, this, _1));
  // clang-format on

  // visualize request
  step_plan_request_vis_pub_.publish(step_plan_request.plan_request);

  if (!isOk(status))
    ROS_INFO("[FootstepPlannerNode] stepPlanRequest:\n%s", toString(status).c_str());
}

void FootstepPlannerNode::goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose)
{
  // get start feet pose
  msgs::FootholdArray start_footholds;
  msgs::ErrorStatus status = feet_pose_generator_->getStartFootholds(start_footholds, goal_pose->header.frame_id);

  if (hasError(status))
  {
    ROS_WARN("[FootstepPlannerNode] Can't obtain start feet pose:\n%s", toString(status).c_str());
    return;
  }
  else if (hasWarning(status))
    ROS_WARN("[FootstepPlannerNode] Warning occured while obtaining start feet pose:\n%s", toString(status).c_str());

  // get start floating bases
  msgs::FloatingBaseArray start_floating_bases;

  if (RobotModel::kinematics())
  {
    /// @todo Implement as optional feature
    // start_floating_bases.push_back(FloatingBase(l3::BaseInfo::MAIN_BODY_IDX, pose, goal_pose->header).toMsg());
  }

  // get goal feet pose
  msgs::FootholdArray goal_footholds;
  status = feet_pose_generator_->getFootholds(goal_footholds, *goal_pose);

  if (hasError(status))
  {
    ROS_WARN("[FootstepPlannerNode] Can't obtain goal feet pose:\n%s", toString(status).c_str());
    return;
  }
  else if (hasWarning(status))
  {
    ROS_WARN("[FootstepPlannerNode] Warning occured while obtaining goal feet pose:\n%s", toString(status).c_str());

    if (status.warning & msgs::ErrorStatus::WARN_NO_TERRAIN_DATA)
    {
      ROS_WARN("[FootstepPlannerNode] Snapping goal heights according to the start state.");

      for (msgs::Foothold& f_goal : goal_footholds)
      {
        for (msgs::Foothold& f_start : start_footholds)
        {
          if (f_goal.idx == f_start.idx)
          {
            f_goal.pose.position.z = f_start.pose.position.z;
            continue;
          }
        }
      }
    }
  }

  footstep_planner_->updateFeet(goal_footholds, msgs::UpdateMode::UPDATE_MODE_MOVE_TO_VALID);

  // get goal floating bases
  msgs::FloatingBaseArray goal_floating_bases;

  // if (RobotModel::kinematics())
  // {
  //   Pose feet_center;
  //   poseMsgToL3(goal_pose->pose, feet_center);

  //   FootholdArray footholds;
  //   footholdArrayMsgToL3(goal_footholds, footholds);

  //   Pose pose = RobotModel::kinematics()->calcBasePose(feet_center, footholds);
  //   /// @todo Implement as optional feature
  //   // goal_floating_bases.push_back(FloatingBase(l3::BaseInfo::MAIN_BODY_IDX, pose, goal_pose->header).toMsg());
  // }

  // request step plan
  msgs::StepPlanRequestService::Request step_plan_request;
  step_plan_request.plan_request.header = goal_pose->header;
  step_plan_request.plan_request.start_footholds = start_footholds;
  step_plan_request.plan_request.goal_footholds = goal_footholds;
  step_plan_request.plan_request.start_floating_bases = start_floating_bases;
  step_plan_request.plan_request.goal_floating_bases = goal_floating_bases;
  step_plan_request.plan_request.start_foot_idx = msgs::StepPlanRequest::AUTO_START_FOOT_IDX;
  step_plan_request.plan_request.start_step_idx = 0;
  step_plan_request.plan_request.planning_mode = WorldModel::instance().isTerrainModelAvailable() ? static_cast<uint8_t>(msgs::StepPlanRequest::PLANNING_MODE_3D) :
                                                                                                    static_cast<uint8_t>(msgs::StepPlanRequest::PLANNING_MODE_2D);
  step_plan_request.plan_request.max_planning_time = 0.0;
  step_plan_request.plan_request.parameter_set_name.data = std::string();

  // clang-format off
  // start planning
  status = footstep_planner_->stepPlanRequest(step_plan_request,
                                              boost::bind(&FootstepPlannerNode::planningResultCallback, this, _1),
                                              boost::bind(&FootstepPlannerNode::planningFeedbackCallback, this, _1));
  // clang-format on

  // visualize request
  step_plan_request_vis_pub_.publish(step_plan_request.plan_request);

  if (!isOk(status))
    ROS_INFO("[FootstepPlannerNode] goalPoseCallback:\n%s", toString(status).c_str());
}

/// --- service calls ---

bool FootstepPlannerNode::stepPlanRequestService(msgs::StepPlanRequestService::Request& req, msgs::StepPlanRequestService::Response& resp)
{
  // generate start feet pose if needed
  if (req.plan_request.start_footholds.empty())
  {
    resp.status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlannerNode",
                                      "stepPlanRequestService: No valid frame_id was given as start pose. Try to use current robot pose as start.");
    resp.status += feet_pose_generator_->getStartFootholds(req.plan_request.start_footholds, req.plan_request.header.frame_id);
  }

  // start planning
  if (!footstep_planner_->stepPlanRequestService(req, resp))
    resp.status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlannerNode", "stepPlanRequestService: Can't call footstep planner service!");

  // visualize request
  step_plan_request_vis_pub_.publish(req.plan_request);

  temp_step_plan_pub_.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(resp.step_plan)));
  step_plan_vis_pub_.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(resp.step_plan)));
  error_status_pub_.publish(msgs::ErrorStatusConstPtr(new msgs::ErrorStatus(resp.status)));

  return true;  // return always true so the message is returned
}

bool FootstepPlannerNode::updateFootService(msgs::UpdateFootService::Request& req, msgs::UpdateFootService::Response& resp)
{
  resp.foot = req.foot;
  resp.status = footstep_planner_->updateFoot(resp.foot, req.update_mode.mode);
  return true;  // return always true so the message is returned
}

bool FootstepPlannerNode::updateFeetService(msgs::UpdateFeetService::Request& req, msgs::UpdateFeetService::Response& resp)
{
  resp.feet = req.feet;
  resp.status = footstep_planner_->updateFeet(resp.feet, req.update_mode.mode);
  return true;  // return always true so the message is returned
}

bool FootstepPlannerNode::updateStepPlanService(msgs::UpdateStepPlanService::Request& req, msgs::UpdateStepPlanService::Response& resp)
{
  resp.step_plan = req.step_plan;
  resp.status = footstep_planner_->updateStepPlan(resp.step_plan, req.update_mode.mode, req.parameter_set_name.data);
  step_plan_vis_pub_.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(resp.step_plan)));
  return true;  // return always true so the message is returned
}

///--- action server calls ---

void FootstepPlannerNode::stepPlanRequestAction(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  // preempt any previous goal if active due to given callback
  footstep_planner_->preemptPlanning();

  boost::recursive_mutex::scoped_lock lock(step_plan_request_as_mutex_);

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
    status += feet_pose_generator_->getStartFootholds(step_plan_request.plan_request.start_footholds, step_plan_request.plan_request.header.frame_id);
  }

  // clang-format off
  // start planning
  status = footstep_planner_->stepPlanRequest(step_plan_request,
                                             boost::bind(&FootstepPlannerNode::planningResultActionCallback, this, _1, boost::ref(as)),
                                             boost::bind(&FootstepPlannerNode::planningFeedbackActionCallback, this, _1, boost::ref(as)),
                                             boost::bind(&FootstepPlannerNode::planningPreemptionActionCallback, this, boost::ref(as)));
  // clang-format on

  // visualize request
  step_plan_request_vis_pub_.publish(step_plan_request.plan_request);

  if (!isOk(status))
    ROS_INFO("[FootstepPlannerNode] stepPlanRequest:\n%s", toString(status).c_str());
}

void FootstepPlannerNode::stepPlanRequestPreempt(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  boost::recursive_mutex::scoped_lock lock(step_plan_request_as_mutex_);

  if (as->isActive())
  {
    footstep_planner_->preemptPlanning();
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
  result.status = footstep_planner_->updateFoot(result.foot, goal->update_mode.mode);

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
  result.status = footstep_planner_->updateFeet(result.feet, goal->update_mode.mode);

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
  result.status = footstep_planner_->updateStepPlan(result.step_plan, goal->update_mode.mode, goal->parameter_set_name.data);
  step_plan_vis_pub_.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(result.step_plan)));

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
