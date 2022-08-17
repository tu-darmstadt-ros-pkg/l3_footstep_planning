#include <l3_footstep_planning_tools/foot_pose_transformer.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

namespace l3_footstep_planning
{
FootPoseTransformer::FootPoseTransformer() {}

bool FootPoseTransformer::initialize(ros::NodeHandle& nh)
{
  RobotDescription::ConstPtr robot_description = RobotDescription::Ptr(new RobotDescription(nh));
  return initialize(robot_description);
}

bool FootPoseTransformer::initialize(RobotDescription::ConstPtr robot_description)
{
  // obtain foot transformation
  for (const FootInfoPair& p : robot_description->getFootInfoMap())
    mutableInstance().offset_map_[p.first] = p.second.link_to_sole_offset;

  return true;
}

void FootPoseTransformer::transform(geometry_msgs::Pose& pose, const Transform& offset)
{
  Pose pose_l3;
  poseMsgToL3(pose, pose_l3);
  transform(pose_l3, offset);
  poseL3ToMsg(pose_l3, pose);
}

bool FootPoseTransformer::transform(Foothold& foothold, const std::string& target_frame)
{
  std::map<FootIndex, Transform>::const_iterator itr = instance().offset_map_.find(foothold.idx);

  if (itr == instance().offset_map_.end())
    return false;

  if (target_frame == "planner")  // to sole frame
  {
    Pose pose = foothold.pose();
    transform(pose, itr->second);
    foothold.setPose(std::move(pose));
    return true;
  }
  else if (target_frame == "robot")  // to tf "foot" frame
  {
    Pose pose = foothold.pose();
    transform(pose, itr->second.inverse());
    foothold.setPose(std::move(pose));
    return true;
  }

  return false;
}

bool FootPoseTransformer::transform(FootholdArray& feet, const std::string& target_frame)
{
  bool result = true;
  for (Foothold& foothold : feet)
    result &= transform(foothold, target_frame);
  return result;
}

bool FootPoseTransformer::transform(FootholdMap& feet, const std::string& target_frame)
{
  bool result = true;
  for (FootholdPair& p : feet)
    result &= transform(p.second, target_frame);
  return result;
}

msgs::ErrorStatus FootPoseTransformer::transform(msgs::Foothold& foothold, const std::string& target_frame)
{
  std::map<FootIndex, Transform>::const_iterator itr = instance().offset_map_.find(foothold.idx);

  if (itr == instance().offset_map_.end())
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootPoseTransformer", "Foot index '" + std::to_string(foothold.idx) + "' is unknown!");

  if (target_frame == "planner")  // to sole frame
  {
    transform(foothold.pose, itr->second);
    return msgs::ErrorStatus();
  }
  else if (target_frame == "robot")  // to tf "foot" frame
  {
    transform(foothold.pose, itr->second.inverse());
    return msgs::ErrorStatus();
  }
  else
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootPoseTransformer", "Target frame '" + target_frame + "' is unknown!");
}

msgs::ErrorStatus FootPoseTransformer::transform(msgs::FootholdArray& feet, const std::string& target_frame)
{
  msgs::ErrorStatus status;
  for (msgs::Foothold& foothold : feet)
    status += transform(foothold, target_frame);
  return status;
}

msgs::ErrorStatus FootPoseTransformer::transform(msgs::FootStepData& foot_step, const std::string& target_frame)
{
  return transform(foot_step.origin, target_frame) + transform(foot_step.target, target_frame);
}

msgs::ErrorStatus FootPoseTransformer::transform(msgs::FootStepDataArray& foot_steps, const std::string& target_frame)
{
  msgs::ErrorStatus status;
  for (msgs::FootStepData& foot_step : foot_steps)
    status += transform(foot_step, target_frame);
  return status;
}

msgs::ErrorStatus FootPoseTransformer::transform(msgs::Step& step, const std::string& target_frame)
{
  msgs::ErrorStatus status;
  status += transform(step.foot_steps, target_frame);
  for (msgs::Foothold& foothold : step.support_feet)
    status += transform(foothold, target_frame);
  return status;
}

msgs::ErrorStatus FootPoseTransformer::transform(msgs::StepArray& step_array, const std::string& target_frame)
{
  msgs::ErrorStatus status;
  for (msgs::Step& step : step_array)
    status += transform(step, target_frame);
  return status;
}

msgs::ErrorStatus FootPoseTransformer::transform(msgs::StepQueue& step_queue, const std::string& target_frame)
{
  msgs::ErrorStatus status;
  for (msgs::Step& step : step_queue.steps)
    status += transform(step, target_frame);
  return status;
}

msgs::ErrorStatus FootPoseTransformer::transform(msgs::StepPlan& step_plan, const std::string& target_frame)
{
  msgs::ErrorStatus status;
  status += transform(step_plan.start, target_frame);
  status += transform(step_plan.goal, target_frame);
  status += transform(step_plan.plan.steps, target_frame);
  return status;
}

/**
 * Handler for convenient service calls
 */

msgs::ErrorStatus transform(msgs::Foothold& foot, ros::ServiceClient& transform_foot_pose_client, const std::string& target_frame)
{
  msgs::TransformFootPoseService transform_service;
  transform_service.request.foot = foot;
  transform_service.request.target_frame.data = target_frame;

  if (!transform_foot_pose_client.call(transform_service.request, transform_service.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "transformToPlannerFrame", "Can't call 'FootPoseTransformer' for foot pose transform!");

  foot = transform_service.response.foot;

  return transform_service.response.status;
}

msgs::ErrorStatus transform(msgs::FootholdArray& feet, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame)
{
  msgs::TransformFeetPosesService transform_service;
  transform_service.request.feet = feet;
  transform_service.request.target_frame.data = target_frame;

  if (!transform_feet_poses_client.call(transform_service.request, transform_service.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "transformToPlannerFrame", "Can't call 'FootPoseTransformer' for feet poses transform!");

  feet = transform_service.response.feet;

  return transform_service.response.status;
}

msgs::ErrorStatus transform(msgs::FootStepData& foot_step, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame)
{
  msgs::ErrorStatus status;
  status += transform(foot_step.origin, transform_feet_poses_client, target_frame);
  status += transform(foot_step.target, transform_feet_poses_client, target_frame);
  return status;
}

msgs::ErrorStatus transform(msgs::FootStepDataArray& foot_steps, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame)
{
  msgs::ErrorStatus status;

  msgs::TransformFeetPosesService transform_service;

  // transform all footholds of each step
  for (const msgs::FootStepData& foot_step : foot_steps)
  {
    transform_service.request.feet.push_back(foot_step.origin);
    transform_service.request.feet.push_back(foot_step.target);
  }

  // call service
  transform_service.request.target_frame.data = target_frame;
  if (!transform_feet_poses_client.call(transform_service.request, transform_service.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "transformToPlannerFrame", "Can't call 'FootPoseTransformer' for step data array transform!");

  // dispatch result
  for (size_t i = 0; i < foot_steps.size(); i++)
  {
    foot_steps[i].origin = transform_service.response.feet[2 * i];
    foot_steps[i].target = transform_service.response.feet[2 * i + 1];
  }

  return status;
}

msgs::ErrorStatus transform(msgs::Step& step, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame)
{
  msgs::ErrorStatus status;
  status += transform(step.foot_steps, transform_feet_poses_client, target_frame);
  status += transform(step.support_feet, transform_feet_poses_client, target_frame);
  return status;
}

msgs::ErrorStatus transform(msgs::StepArray& step_array, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame)
{
  msgs::ErrorStatus status;
  for (msgs::Step& step : step_array)
    status += transform(step, transform_feet_poses_client, target_frame);
  return status;
}

msgs::ErrorStatus transform(msgs::StepQueue& step_queue, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame)
{
  msgs::ErrorStatus status;
  for (msgs::Step& step : step_queue.steps)
    status += transform(step, transform_feet_poses_client, target_frame);
  return status;
}

msgs::ErrorStatus transform(msgs::StepPlan& step_plan, ros::ServiceClient& transform_step_plan_client, const std::string& target_frame)
{
  msgs::TransformStepPlanService transform_service;
  transform_service.request.step_plan = step_plan;
  transform_service.request.target_frame.data = target_frame;

  if (!transform_step_plan_client.call(transform_service.request, transform_service.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "transformToPlannerFrame", "Can't call 'FootPoseTransformer' for step plan transform!");

  step_plan = transform_service.response.step_plan;

  return transform_service.response.status;
}
}  // namespace l3_footstep_planning
