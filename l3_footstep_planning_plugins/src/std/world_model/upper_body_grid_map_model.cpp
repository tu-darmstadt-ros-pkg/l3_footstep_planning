#include <l3_footstep_planning_plugins/std/world_model/upper_body_grid_map_model.h>

#include <l3_plugins/robot_model.h>

namespace l3_footstep_planning
{
UpperBodyGridMapModel::UpperBodyGridMapModel(const std::string& name)
  : GridMapModel(name)
{}

bool UpperBodyGridMapModel::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GridMapModel::loadParams(params))
    return false;

  // get upper body dimensions
  BaseInfo base_info;
  RobotModel::description()->getBaseInfo(BaseInfo::MAIN_BODY_IDX, base_info);
  upper_body_size_ = base_info.size;
  upper_body_offset_ = base_info.link_to_feet_center_offset;

  return true;
}

bool UpperBodyGridMapModel::isAccessible(const Foothold& /*foothold*/) const
{
  // We can't make any checks with a single foot pose
  return true;
}

bool UpperBodyGridMapModel::isAccessible(const State& state) const
{
  if (!occupancy_grid_map_)
  {
    ROS_ERROR_THROTTLE(10, "[UpperBodyGridMapModel] No upper body level grid map available yet.");
    return true;
  }

  const Pose& center = state.getFeetCenter();

  // determine shift of polygon based on foot orientation
  double yaw = center.yaw();
  double cos_theta = cos(yaw);
  double sin_theta = sin(yaw);
  double shift_x = cos_theta * upper_body_offset_.x() - sin_theta * upper_body_offset_.y();
  double shift_y = sin_theta * upper_body_offset_.x() + cos_theta * upper_body_offset_.y();

  return !checkCollision(center.x() + shift_x, center.y() + shift_y, cos_theta, sin_theta, upper_body_size_.x(), upper_body_size_.y());
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::UpperBodyGridMapModel, l3_footstep_planning::WorldModelPlugin)
