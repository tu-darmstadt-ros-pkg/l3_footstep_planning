#include <l3_footstep_planning_plugins/std/world_model/foot_grid_map_model.h>

#include <l3_plugins/robot_model.h>

namespace l3_footstep_planning
{
FootGridMapModel::FootGridMapModel(const std::string& name)
  : GridMapModel(name)
{}

bool FootGridMapModel::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GridMapModel::loadParams(params))
    return false;

  foot_size_map_.clear();
  for (const FootInfoPair& p : RobotModel::description()->getFootInfoMap())
    foot_size_map_.emplace(p.first, p.second.size);

  return true;
}

bool FootGridMapModel::isAccessible(const Foothold& foothold) const
{
  if (!occupancy_grid_map_)
  {
    ROS_ERROR_THROTTLE(10, "[FootGridMapModel] No ground level grid map available yet.");
    return true;
  }

  std::map<FootIndex, Vector3>::const_iterator itr = foot_size_map_.find(foothold.idx);
  ROS_ASSERT(itr != foot_size_map_.end());
  const Vector3& size = itr->second;

  return !checkCollision(foothold.x(), foothold.y(), cos(foothold.yaw()), sin(foothold.yaw()), size.x(), size.y());
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::FootGridMapModel, l3_footstep_planning::WorldModelPlugin)
