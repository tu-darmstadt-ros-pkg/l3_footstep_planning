#include <l3_footstep_planning_tools/feet_pose_generator.h>

#include <vigir_generic_params/parameter_manager.h>

#include <l3_libs/conversions/l3_msg_conversions.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_libs/helper.h>

#include <l3_footstep_planning_tools/foot_pose_transformer.h>

namespace l3_footstep_planning
{
FeetPoseGenerator::FeetPoseGenerator(ros::NodeHandle& nh)
  : has_robot_pose_(false)
{
  // initialize foot pose transformer
  FootPoseTransformer::initialize(nh);
}

void FeetPoseGenerator::setRobotPose(const geometry_msgs::PoseStampedConstPtr& robot_pose)
{
  this->robot_pose_.header = robot_pose->header;
  this->robot_pose_ = *robot_pose;
  has_robot_pose_ = true;
}

void FeetPoseGenerator::setRobotPoseWithCovariance(const geometry_msgs::PoseWithCovarianceStampedConstPtr& robot_pose)
{
  this->robot_pose_.header = robot_pose->header;
  this->robot_pose_.pose = robot_pose->pose.pose;
  has_robot_pose_ = true;
}

void FeetPoseGenerator::setTerrainModel(const l3_terrain_modeling::TerrainModelMsg::ConstPtr& terrain_model)
{
  // update terrain model
  if (this->terrain_model_)
    this->terrain_model_->fromMsg(*terrain_model);
  else
    this->terrain_model_.reset(new l3_terrain_modeling::TerrainModel(*terrain_model));
}

msgs::ErrorStatus FeetPoseGenerator::generateFeetPose(const msgs::FeetPoseRequest& request, msgs::FootholdArray& feet)
{
  msgs::ErrorStatus status;

  // strip '/'
  std::string request_frame_id = request.header.frame_id;
  strip(request_frame_id, '/');

  if (request.flags & msgs::FeetPoseRequest::FLAG_CURRENT)
  {
    // try to get current feet pose
    if (getCurrentFeetPose(feet, request_frame_id))
    {
      return status;
    }
    // project robot (pelvis) pose to the ground
    else if (has_robot_pose_)
    {
      std::string robot_pose_frame_id = strip_const(robot_pose_.header.frame_id, '/');
      if (request_frame_id != robot_pose_frame_id)
        status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FeetPoseGenerator",
                                     "generateFeetPose: Frame ID of robot pose ('" + robot_pose_frame_id + "') and request ('" + request_frame_id +
                                         "') mismatch, automatic transformation is not implemented yet!");

      feet = generateFeetPose(robot_pose_);
      // pelvisToGroundTransform(feet);
    }
    else
    {
      return status + ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FeetPoseGenerator", "generateFeetPose: No robot pose available for handling FLAG_CURRENT!", false);
    }
  }
  else
  {
    geometry_msgs::PoseStamped pose;
    pose.header = request.header;
    pose.pose = request.pose;
    feet = generateFeetPose(pose);
  }

  if (request.flags & msgs::FeetPoseRequest::FLAG_PELVIS_FRAME)
  {
    pelvisToGroundTransform(feet);
  }

  if (request.flags & msgs::FeetPoseRequest::FLAG_3D)
  {
    msgs::ErrorStatus temp_status;
    msgs::FootholdArray temp_feet = feet;

    temp_status = updateFeetPose(temp_feet);
    if (hasError(temp_status))
    {
      if (hasError(temp_status, msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL))
        status += ErrorStatusWarning(msgs::ErrorStatus::WARN_NO_TERRAIN_DATA, "FeetPoseGenerator", "generateFeetPose: No terrain model available!", false);
      else
        status += temp_status;
    }
    else if (hasWarning(temp_status, msgs::ErrorStatus::WARN_NO_TERRAIN_DATA))
    {
      status += ErrorStatusWarning(msgs::ErrorStatus::WARN_NO_TERRAIN_DATA, "FeetPoseGenerator", "generateFeetPose: Couldn't determine neither 3D nor height of feet!", false);
    }
    else
    {
      status += temp_status;
      feet = temp_feet;
    }
  }

  if (request.flags & msgs::FeetPoseRequest::FLAG_CURRENT_Z)
  {
    msgs::FootholdArray current_feet;
    if (!getCurrentFeetPose(current_feet, request_frame_id))
      status += ErrorStatusWarning(msgs::ErrorStatus::WARN_NO_TERRAIN_DATA, "FeetPoseGenerator", "generateFeetPose: Couldn't determine height of current feet!");
  }

  return status;
}

msgs::ErrorStatus FeetPoseGenerator::updateFeetPose(msgs::FootholdArray& feet)
{
  if (!terrain_model_ || !terrain_model_->hasTerrainModel())
    return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL, "FeetPoseGenerator", "updateFeetPose: No terrain model available!", false);

  msgs::ErrorStatus status;

  FootPoseTransformer::transformToPlannerFrame(feet);

  for (msgs::Foothold& foothold : feet)
  {
    if (!terrain_model_->update3DData(foothold.pose) && !terrain_model_->getHeight(foothold.pose.position.x, foothold.pose.position.y, foothold.pose.position.z))
      status += ErrorStatusWarning(msgs::ErrorStatus::WARN_NO_TERRAIN_DATA, "FeetPoseGenerator",
                                   "updateFeetPose: Couldn't determine neither 3D nor height of foot_idx " + std::to_string(foothold.idx) + "!", false);
  }

  FootPoseTransformer::transformToRobotFrame(feet);

  return status;
}

bool FeetPoseGenerator::getCurrentFeetPose(msgs::FootholdArray& feet, const std::string& request_frame)
{
  feet.clear();

  for (const FootInfoPair& p : RobotModel::description()->getFootInfoMap())
  {
    const FootInfo& foot_info = p.second;
    if (tf_listener_.canTransform(request_frame, foot_info.link, ros::Time(0)))
    {
      msgs::Foothold foothold;
      tf::StampedTransform t;

      tf_listener_.lookupTransform(request_frame, foot_info.link, ros::Time(0), t);
      tf::poseTFToMsg(t, foothold.pose);
      foothold.header.frame_id = request_frame;
      foothold.header.stamp = t.stamp_;
      foothold.idx = foot_info.idx;
      feet.push_back(foothold);
    }
    else
      return false;
  }
  return true;
}

msgs::FootholdArray FeetPoseGenerator::generateFeetPose(const geometry_msgs::PoseStamped& pose)
{
  msgs::FootholdArray feet;

  // if kinematics are defined use it for more precise calculation
  if (RobotModel::kinematics())
  {
    // read resolution
    const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterManager::getActive();
    DiscreteResolution planner_res = DiscreteResolution(params.getSubset("resolution"));

    // determine base pose
    Pose feet_center;
    poseMsgToL3(pose.pose, feet_center);
    Pose base_pose = RobotModel::kinematics()->calcStaticBasePose(feet_center);
    footholdArrayL3ToMsg(getNeutralStance(FloatingBase(BaseInfo::MAIN_BODY_IDX, base_pose), planner_res), feet);
  }
  // otherwise use fallback to feet_center
  else
  {
    Pose feet_center;
    poseMsgToL3(pose.pose, feet_center);
    footholdArrayL3ToMsg(RobotModel::description()->getNeutralStance(feet_center), feet);
  }

  for (msgs::Foothold& foot : feet)
    foot.header = pose.header;

  FootPoseTransformer::transformToRobotFrame(feet);

  return feet;
}

void FeetPoseGenerator::pelvisToGroundTransform(msgs::FootholdArray& feet) const
{
  /// @TODO: Transform pelvis_to_feet_center to world frame!
  /// @TODO: use RobotModelPlugin
  //  feet[0].pose.position.x += pelvis_to_feet_center.x;
  //  feet[0].pose.position.y += pelvis_to_feet_center.y;
  //  feet[0].pose.position.z += pelvis_to_feet_center.z;

  //  feet[1].pose.position.x += pelvis_to_feet_center.x;
  //  feet[1].pose.position.y += pelvis_to_feet_center.y;
  //  feet[1].pose.position.z += pelvis_to_feet_center.z;

  //  FootPoseTransformer::transformToRobotFrame(feet);

  ROS_ERROR("[FeetPoseGenerator] pelvisToGroundTransform not implemented yet!");
}

/**
 * Handler for convenient service calls
 */

msgs::ErrorStatus determineFootholds(msgs::FootholdArray& footholds, ros::ServiceClient& generate_feet_pose_client, const geometry_msgs::PoseStamped& pose)
{
  msgs::GenerateFeetPoseService feet_pose_service;
  feet_pose_service.request.request.header = pose.header;
  feet_pose_service.request.request.pose = pose.pose;
  feet_pose_service.request.request.flags = msgs::FeetPoseRequest::FLAG_3D;

  if (!generate_feet_pose_client.call(feet_pose_service.request, feet_pose_service.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "determineFootholds", "Can't call 'FeetPoseGenerator'!");

  footholds = feet_pose_service.response.feet;

  return feet_pose_service.response.status;
}

msgs::ErrorStatus determineStartFootholds(msgs::FootholdArray& start_footholds, ros::ServiceClient& generate_feet_pose_client, const std_msgs::Header& header)
{
  msgs::ErrorStatus status;

  // get start feet pose
  msgs::GenerateFeetPoseService feet_pose_service;
  feet_pose_service.request.request.header = header;
  feet_pose_service.request.request.flags = msgs::FeetPoseRequest::FLAG_CURRENT | msgs::FeetPoseRequest::FLAG_3D;

  if (!generate_feet_pose_client.call(feet_pose_service.request, feet_pose_service.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "determineStartFeetPose", "Can't call 'FeetPoseGenerator'!", true, 1.0);

  // if real position could not be determined, default to origin
  if (hasError(feet_pose_service.response.status))
  {
    // status += feet_pose_service.response.status;
    status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "determineStartFeetPose", "Can't obtain start feet pose; defaulting to origin.", true, 1.0);

    feet_pose_service.request.request.pose = geometry_msgs::Pose();
    feet_pose_service.request.request.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    feet_pose_service.request.request.flags = msgs::FeetPoseRequest::FLAG_3D;

    if (!generate_feet_pose_client.call(feet_pose_service.request, feet_pose_service.response))
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "determineStartFeetPose", "Can't call 'FeetPoseGenerator'!", true, 1.0);

    if (hasError(feet_pose_service.response.status))
    {
      status += feet_pose_service.response.status;
      return status;
    }
  }
  else if (hasWarning(feet_pose_service.response.status))
    status += feet_pose_service.response.status;

  start_footholds = feet_pose_service.response.feet;
  return status;
}
}  // namespace l3_footstep_planning
