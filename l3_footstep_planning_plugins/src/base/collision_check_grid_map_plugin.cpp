#include <l3_footstep_planning_plugins/base/collision_check_grid_map_plugin.h>

namespace l3_footstep_planning
{
CollisionCheckGridMapPlugin::CollisionCheckGridMapPlugin(const std::string& name)
  : WorldModelPlugin(name)
  , occ_thresh_(70)
{}

bool CollisionCheckGridMapPlugin::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!WorldModelPlugin::initialize(params))
    return false;

  std::string topic;
  getParam("topic", topic, std::string("grid_map"), true);
  occupancy_grid_map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(topic, 1, &CollisionCheckGridMapPlugin::mapCallback, this);

  getParam("occupancy_threshold", (int&)occ_thresh_, (int&)occ_thresh_, true);

  return true;
}

void CollisionCheckGridMapPlugin::reset()
{
  WorldModelPlugin::reset();

  occupancy_grid_map_.reset();
}

bool CollisionCheckGridMapPlugin::isCollisionCheckAvailable() const { return WorldModelPlugin::isCollisionCheckAvailable() && occupancy_grid_map_ != nullptr; }

bool CollisionCheckGridMapPlugin::isAccessible(const Foothold& foothold) const
{
  if (!occupancy_grid_map_)
  {
    ROS_ERROR_THROTTLE(10, "[CollisionCheckGridMapPlugin] No grid map available yet.");
    return true;
  }

  int idx = 0;
  if (getGridMapIndex(*occupancy_grid_map_, foothold.x(), foothold.y(), idx))
    return occupancy_grid_map_->data.at(idx) <= occ_thresh_;

  return false;
}

void CollisionCheckGridMapPlugin::setOccupancyThreshold(unsigned char thresh) { occ_thresh_ = static_cast<int8_t>(thresh); }

void CollisionCheckGridMapPlugin::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map)
{
  if (isLocked())
    return;

  this->occupancy_grid_map_ = occupancy_grid_map;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::CollisionCheckGridMapPlugin, l3_footstep_planning::WorldModelPlugin)
