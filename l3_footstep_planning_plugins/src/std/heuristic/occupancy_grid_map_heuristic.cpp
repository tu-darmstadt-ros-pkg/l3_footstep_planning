#include <l3_footstep_planning_plugins/std/heuristic/occupancy_grid_map_heuristic.h>

namespace l3_footstep_planning
{
OccupancyGridMapHeuristic::OccupancyGridMapHeuristic()
  : HeuristicPlugin("occupancy_grid_map_heuristic")
{}

bool OccupancyGridMapHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::loadParams(params))
    return false;

  getParam("grid_map_topic", grid_map_topic_, std::string());
  return true;
}

bool OccupancyGridMapHeuristic::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::initialize(params))
    return false;

  // subscribe topics
  occupancy_grid_map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(grid_map_topic_, 1, &OccupancyGridMapHeuristic::mapCallback, this);

  return true;
}

void OccupancyGridMapHeuristic::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map)
{
  UniqueLock lock(grid_map_shared_mutex_);
  distance_map_.setMap(occupancy_grid_map);
}

double OccupancyGridMapHeuristic::getHeuristicValue(const Foothold& from, const Foothold& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  SharedLock lock(grid_map_shared_mutex_);

  double d = distance_map_.distanceMapAt(from.x(), from.y());
  if (d < 0.0)
    return 0.0;

  if (d < 0.01)
    return max_heuristic_value_;

  return 5.0 * std::max(0.0, 0.5 - d);
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::OccupancyGridMapHeuristic, l3_footstep_planning::HeuristicPlugin)
