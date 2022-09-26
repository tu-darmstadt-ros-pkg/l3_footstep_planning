#include <l3_footstep_planning_vis_tools/std/inverse_kinematics_vis.h>

#include <kdl_parser/kdl_parser.hpp>

#include <l3_libs/conversions/l3_tf_conversions.h>
#include <l3_libs/helper.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_tools/foot_pose_transformer.h>

namespace l3_footstep_planning
{
InverseKinematicsVis::InverseKinematicsVis()
  : PlanningVisPlugin("inverse_kinematics_vis")
  , current_step_idx_(-1)
{}

bool InverseKinematicsVis::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PlanningVisPlugin::initialize(params))
    return false;

  /// check if robot model and plugins are available

  // check for valid robot description
  if (!RobotModel::description())
  {
    ROS_ERROR("[InverseKinematicsVis] initialize: No robot description available by RobotModel. Cannot initialize!");
    return false;
  }

  // check for kinematics plugin
  if (!RobotModel::kinematics())
  {
    ROS_ERROR("[InverseKinematicsVis] initialize: Failed to retrieve KinematicsPlugin. Ensure to load a KinematicPlugin!");
    return false;
  }

  def_leg_joint_states_.clear();
  vis_joint_states_.clear();

  /// get params

  // get kdl tree from robot description
  std::string robot_description = param("robot_description", std::string("robot_description"), true);
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromParam(robot_description, kdl_tree))
  {
    ROS_ERROR("[InverseKinematicsVis] initialize: Failed to construct KDL tree from robot description");
    return false;
  }

  // get base to root transform
  BaseInfo base_info;
  RobotModel::description()->getBaseInfo(BaseInfo::MAIN_BODY_IDX, base_info);
  const std::string& base_link = base_info.link;
  const std::string& root_link = RobotModel::kinematics()->getRootLink();
  if (!RobotModel::kinematics()->calcStaticTransformForChain(base_link, root_link, base_to_root_))
    ROS_WARN("[%s] initialize: Could not determine transform from base ('%s') to root ('%s'). Using identity transform.", getName().c_str(), base_link.c_str(), root_link.c_str());

  // get other parameters
  tf_prefix_ = param("tf_prefix", std::string("planning_vis"), true);
  visualize_feedback_ = param("visualize_feedback", false, true);
  animate_time_scale_ = param("animate_time_scale", 1.0, true);

  /// init ROS API stuff

  // init joint state fields
  for (const std::pair<std::string, KDL::TreeElement>& p : kdl_tree.getSegments())
  {
    if (p.second.segment.getJoint().getType() != KDL::Joint::None)
      cur_joint_states_[p.second.segment.getJoint().getName().c_str()] = 0.0;
  }

  // get default joint values
  std::vector<std::string> joint_names = param("joint_names", std::vector<std::string>(), true);
  std::vector<double> joint_values = param("joint_values", std::vector<double>(), true);

  if (joint_names.size() != joint_values.size())
    ROS_ERROR("[%s] initialize: 'joint_names' does not have same length as 'joint_values'!", getName().c_str());
  else
  {
    for (size_t i = 0; i < joint_names.size(); i++)
      cur_joint_states_[joint_names[i]] = joint_values[i];
  }

  if (!calcNeutralStanceIK(def_leg_joint_states_))
    ROS_WARN("[%s] initialize: Calcuation of neutral stance joint states failed.", getName().c_str());

  // subscribe topics
  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>(param("joint_states_topic", std::string("joint_states"), true), 1, &InverseKinematicsVis::jointStateCB, this);
  rviz_visual_tools_gui_sub_ = nh_.subscribe<sensor_msgs::Joy>("/rviz_visual_tools_gui", 1, &InverseKinematicsVis::rvizVisualToolsGuiCB, this);

  // create robot state publisher
  robot_state_publisher_.reset(new robot_state_publisher::RobotStatePublisher(kdl_tree));

  // start timer
  double rate = param("tf_publish_rate", 10.0, true);
  if (rate > 0.0)
    publish_tf_timer_ = nh_.createTimer(ros::Rate(rate), &InverseKinematicsVis::publishTF, this);

  // instantiate timer, but not started
  animation_timer_ = nh_.createTimer(ros::Duration(1.0), &InverseKinematicsVis::animate, this, true, false);

  return true;
}

void InverseKinematicsVis::clear()
{
  current_step_plan_.clear();
  current_step_idx_ = -1;
}

void InverseKinematicsVis::visualize(const msgs::StepPlanRequest& step_plan_request)
{
  if (!step_plan_request.goal_footholds.empty() && !step_plan_request.goal_floating_bases.empty())
  {
    FootholdArray footholds;
    footholdArrayMsgToL3(step_plan_request.goal_footholds, footholds);

    FloatingBaseArray floating_bases;
    floatingBaseArrayMsgToL3(step_plan_request.goal_floating_bases, floating_bases);

    FloatingBaseMap floating_bases_map;
    for (const FloatingBase& fb : floating_bases)
      floating_bases_map[fb.idx] = fb;

    visualize(footholds, floating_bases_map, step_plan_request.header.frame_id);
  }
}

void InverseKinematicsVis::visualize(const msgs::PlanningFeedback& planning_feedback)
{
  if (!visualize_feedback_)
    return;

  visualize(Step(planning_feedback.last_visited_step), planning_feedback.header.frame_id);
}

void InverseKinematicsVis::visualize(const StepPlan& step_plan)
{
  current_step_plan_ = step_plan;

  if (current_step_plan_.empty())
    return;

  Step::ConstPtr step = current_step_plan_.getfirstStep();
  current_step_idx_ = step->getStepIndex();
  visualize(*step, current_step_plan_.getHeader().frame_id);

  // start animation
  // take step duration for next step index to simulate movement timing to this next state
  Step::ConstPtr next_step = current_step_plan_.getSteps().getNextStep(current_step_idx_);
  if (next_step)
    scheduleAnimationStep(next_step->getStepDuration());
}

void InverseKinematicsVis::visualize(const Step& step, const std::string& frame_id)
{
  FootholdArray footholds;
  for (const Step::FootStep::MovingDataPair& p : step.footStep().getMovingLinks())
    footholds.push_back(*p.second->target);

  for (const Step::FootStep::NonMovingDataPair& p : step.footStep().getNonMovingLinks())
    footholds.push_back(*p.second);

  FloatingBaseMap floating_bases;
  for (const Step::BaseStep::MovingDataPair& p : step.baseStep().getMovingLinks())
    floating_bases[p.first] = (*p.second->target);

  for (const Step::BaseStep::NonMovingDataPair& p : step.baseStep().getNonMovingLinks())
    floating_bases[p.first] = (*p.second);

  // in case of the planner does not use floating bases, generate one based on central body estimate
  if (floating_bases.find(BaseInfo::MAIN_BODY_IDX) == floating_bases.end())
  {
    ROS_WARN("[%s] Step %u has no main body floating base.", getName().c_str(), step.getStepIndex());
    return;
  }

  visualize(footholds, floating_bases, frame_id);
}

void InverseKinematicsVis::visualize(const FootholdArray& footholds, const FloatingBaseMap& floating_bases, const std::string& frame_id)
{
  /// @todo Implement visualization for multi floating bases
  FloatingBaseMap::const_iterator itr = floating_bases.find(BaseInfo::MAIN_BODY_IDX);
  if (itr == floating_bases.end())
  {
    ROS_WARN("[%s] Cannot visualize due to missing floating base.", getName().c_str());
    return;
  }

  FloatingBase floating_base = itr->second;

  if (frame_id.empty())
    ROS_WARN("[%s] visualize(...) called with empty frame id!", getName().c_str());

  if (animation_timer_.hasStarted())
    animation_timer_.stop();

  if (!RobotModel::kinematics())
    return;

  vis_joint_states_ = cur_joint_states_;

  // world frame -> root transform; will be published later
  world_to_root_.header.frame_id = frame_id;
  world_to_root_.child_frame_id = ros::names::append(tf_prefix_, RobotModel::kinematics()->getRootLink());
  transformL3ToMsg(base_to_root_ * floating_base.pose(), world_to_root_.transform);

  // compute ik solution for each foothold
  for (const Foothold& foothold : footholds)
  {
    // ignore foot without leg info
    if (!RobotModel::description()->hasLegInfo(foothold.idx))
    {
      ROS_DEBUG("[%s] visualize(...): Missing leg info for foot idx %u!", getName().c_str(), foothold.idx);
      continue;
    }

    const LegInfo& leg_info = RobotModel::description()->getLegInfo(foothold.idx);

    std::vector<double> q;
    if (calcLegIK(floating_base.pose(), foothold, q))
    {
      ROS_ASSERT(leg_info.joints.size() <= q.size());
      for (size_t i = 0; i < leg_info.joints.size(); i++)
        vis_joint_states_[leg_info.joints[i]] = q[i];
    }
    else
    {
      for (size_t i = 0; i < leg_info.joints.size(); i++)
        vis_joint_states_[leg_info.joints[i]] = 0.0;
    }
  }
}

bool InverseKinematicsVis::calcLegIK(const Pose& base_pose, const Foothold& foothold, std::vector<double>& q) const
{
  const LegInfo& leg_info = RobotModel::description()->getLegInfo(foothold.idx);

  std::map<LegIndex, std::vector<double>>::const_iterator joint_states_itr = def_leg_joint_states_.find(leg_info.idx);
  if (joint_states_itr == def_leg_joint_states_.end())
    return RobotModel::kinematics()->calcLegIK(base_pose, foothold, q);
  else
    return RobotModel::kinematics()->calcLegIK(base_pose, foothold, joint_states_itr->second, q);
}

bool InverseKinematicsVis::calcNeutralStanceIK(std::map<LegIndex, std::vector<double>>& leg_joint_states) const
{
  return RobotModel::kinematics()->calcNeutralStanceIK(RobotModel::kinematics()->calcStaticFeetCenterToBase(), leg_joint_states);
}

void InverseKinematicsVis::jointStateCB(const sensor_msgs::JointStateConstPtr& msg)
{
  for (size_t i = 0; i < msg->name.size(); i++)
    cur_joint_states_[msg->name[i]] = msg->position[i];
}

void InverseKinematicsVis::rvizVisualToolsGuiCB(const sensor_msgs::JoyConstPtr& msg)
{
  if (current_step_plan_.empty())
    return;

  Step::ConstPtr step;

  if (msg->buttons.size() >= 5)  // assuming rviz visual tools GUI
  {
    if (msg->buttons[1])  // "Next" -> Previous
      step = current_step_plan_.getSteps().getPrevStep(current_step_idx_);
    else if (msg->buttons[2])  // "Continue" -> Next
      step = current_step_plan_.getSteps().getNextStep(current_step_idx_);
    else if (msg->buttons[3])  // "Break" -> First
    {
      visualize(current_step_plan_);
      return;
    }
    else if (msg->buttons[4])  // "Stop" -> Last
      step = current_step_plan_.getLastStep();
  }

  if (!step)
    return;

  current_step_idx_ = step->getStepIndex();
  visualize(*step, current_step_plan_.getHeader().frame_id);
}

void InverseKinematicsVis::publishTF(const ros::TimerEvent& /*event*/)
{
  if (robot_state_publisher_ && !vis_joint_states_.empty())
  {
    ros::Time current_time = ros::Time().now();

    // prevent duplicate time stamps
    if (world_to_root_.header.stamp == current_time)
      return;

    world_to_root_.header.stamp = current_time;
    tf_broadcaster_.sendTransform(world_to_root_);

    // publish moving joints
    robot_state_publisher_->publishTransforms(vis_joint_states_, current_time, tf_prefix_);

    // publish fixed joints
    robot_state_publisher_->publishFixedTransforms(tf_prefix_, true);
  }
}

void InverseKinematicsVis::animate(const ros::TimerEvent& /*event*/)
{
  if (current_step_plan_.empty())
    return;

  Step::ConstPtr step = current_step_plan_.getSteps().getNextStep(current_step_idx_);
  if (!step)
    return;

  current_step_idx_ = step->getStepIndex();
  visualize(*step, current_step_plan_.getHeader().frame_id);

  // take step duration for next step index to simulate movement timing to this next state
  Step::ConstPtr next_step = current_step_plan_.getSteps().getNextStep(current_step_idx_);
  if (next_step)
    scheduleAnimationStep(next_step->getStepDuration());
}

void InverseKinematicsVis::scheduleAnimationStep(double time_step)
{
  if (animate_time_scale_ > 0.0)
  {
    animation_timer_.setPeriod(ros::Duration(time_step * animate_time_scale_));
    animation_timer_.start();
  }
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::InverseKinematicsVis, l3_footstep_planning::PlanningVisPlugin)
