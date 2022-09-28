#include <l3_footstep_planning_tools/feet_pose_generator_client.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>
#include <l3_libs/conversions/l3_msg_conversions.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_tools/foot_pose_transformer.h>

namespace l3_footstep_planning
{
FeetPoseGeneratorClient::FeetPoseGeneratorClient(ros::NodeHandle& nh)
{
  // initialize  robot model
  RobotModel::initialize(nh);

  // initialize foot pose transformer
  FootPoseTransformer::initialize(nh);

  // service clients
  generate_feet_pose_client_ = nh.serviceClient<l3_footstep_planning_msgs::GenerateFeetPoseService>("generate_feet_pose");
  update_feet_client_ = nh.serviceClient<l3_footstep_planning_msgs::UpdateFeetService>("update_feet");
}

msgs::ErrorStatus FeetPoseGeneratorClient::getFootholds(msgs::FootholdArray& footholds, const FloatingBase& floating_base, const std::string& frame_id)
{
  Pose feet_center = floating_base.pose() * RobotModel::kinematics()->calcStaticFeetCenterToBase().inverse();
  return getFootholds(footholds, feet_center, frame_id);
}

msgs::ErrorStatus FeetPoseGeneratorClient::getFootholds(FootholdArray& footholds, const FloatingBase& floating_base, const std::string& frame_id)
{
  msgs::FootholdArray footholds_msg;
  msgs::ErrorStatus status = getFootholds(footholds_msg, floating_base, frame_id);
  footholdArrayMsgToL3(footholds_msg, footholds);
  return status;
}

msgs::ErrorStatus FeetPoseGeneratorClient::getFootholds(msgs::FootholdArray& footholds, const Pose& feet_center, const std::string& frame_id)
{
  geometry_msgs::PoseStamped feet_center_msg;
  feet_center_msg.header.frame_id = frame_id;
  poseL3ToMsg(feet_center, feet_center_msg.pose);
  return getFootholds(footholds, feet_center_msg);
}

msgs::ErrorStatus FeetPoseGeneratorClient::getFootholds(FootholdArray& footholds, const Pose& feet_center, const std::string& frame_id)
{
  msgs::FootholdArray footholds_msg;
  msgs::ErrorStatus status = getFootholds(footholds_msg, feet_center, frame_id);
  footholdArrayMsgToL3(footholds_msg, footholds);
  return status;
}

msgs::ErrorStatus FeetPoseGeneratorClient::getFootholds(msgs::FootholdArray& footholds, const geometry_msgs::PoseStamped& feet_center)
{
  return determineFootholds(footholds, feet_center);
}

msgs::ErrorStatus FeetPoseGeneratorClient::getFootholds(FootholdArray& footholds, const geometry_msgs::PoseStamped& feet_center)
{
  msgs::FootholdArray footholds_msg;
  msgs::ErrorStatus status = getFootholds(footholds_msg, feet_center);
  footholdArrayMsgToL3(footholds_msg, footholds);
  return status;
}

msgs::ErrorStatus FeetPoseGeneratorClient::getStartFootholds(msgs::FootholdArray& footholds, const std::string& frame_id)
{
  std_msgs::Header header;
  header.frame_id = frame_id;
  return determineStartFootholds(footholds, header);
}

msgs::ErrorStatus FeetPoseGeneratorClient::getStartFootholds(FootholdArray& footholds, const std::string& frame_id)
{
  msgs::FootholdArray footholds_msg;
  msgs::ErrorStatus status = getStartFootholds(footholds_msg, frame_id);
  footholdArrayMsgToL3(footholds_msg, footholds);
  return status;
}

msgs::ErrorStatus FeetPoseGeneratorClient::determineFootholds(msgs::FootholdArray& footholds, const geometry_msgs::PoseStamped& pose)
{
  msgs::GenerateFeetPoseService feet_pose_srv_req;
  feet_pose_srv_req.request.request.header = pose.header;
  feet_pose_srv_req.request.request.pose = pose.pose;
  feet_pose_srv_req.request.request.flags = msgs::FeetPoseRequest::FLAG_3D;

  if (!generate_feet_pose_client_.call(feet_pose_srv_req.request, feet_pose_srv_req.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "determineFootholds", "Can't call 'FeetPoseGenerator'!");

  footholds = feet_pose_srv_req.response.feet;

  return feet_pose_srv_req.response.status;
}

msgs::ErrorStatus FeetPoseGeneratorClient::determineStartFootholds(msgs::FootholdArray& footholds, const std_msgs::Header& header)
{
  msgs::ErrorStatus status;

  // get start feet pose
  msgs::GenerateFeetPoseService feet_pose_srv_req;
  feet_pose_srv_req.request.request.header = header;
  feet_pose_srv_req.request.request.flags = msgs::FeetPoseRequest::FLAG_CURRENT | msgs::FeetPoseRequest::FLAG_3D;

  if (!generate_feet_pose_client_.call(feet_pose_srv_req.request, feet_pose_srv_req.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "determineStartFeetPose", "Can't call 'FeetPoseGenerator'!", true, 1.0);

  // if real position could not be determined, default to origin
  if (hasError(feet_pose_srv_req.response.status))
  {
    // status += feet_pose_service.response.status;
    status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "determineStartFeetPose", "Can't obtain start feet pose; defaulting to origin.", true, 1.0);

    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    status += determineFootholds(footholds, pose);
  }
  else
  {
    footholds = feet_pose_srv_req.response.feet;
    status += feet_pose_srv_req.response.status;
  }

  return status;
}
}  // namespace l3_footstep_planning
