#include <l3_footstep_planning_plugins/std/use_mask_generator/occ_grid_use_mask_generator.h>

namespace l3_footstep_planning
{
OccGridUseMaskGenerator::OccGridUseMaskGenerator()
  : UseMaskGeneratorPlugin("occ_grid_use_mask_generator")
{}

bool OccGridUseMaskGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!UseMaskGeneratorPlugin::loadParams(params))
    return false;

  min_distance_ = param("min_distance", 0.0);
  return true;
}

bool OccGridUseMaskGenerator::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!UseMaskGeneratorPlugin::initialize(params))
    return false;

  std::string topic;
  getParam("topic", topic, std::string("grid_map"), true);
  occupancy_grid_map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(topic, 1, &OccGridUseMaskGenerator::mapCallback, this);

  return true;
}

UseMask OccGridUseMaskGenerator::determineStateGenerationUseMask(const PlanningState& state, const State& /*start*/, const State& /*goal*/) const
{
  const l3::Pose& pose = state.getState()->getFeetCenter();
  float dist = distance_map_.distanceMapAt(pose.x(), pose.y());

  if (dist < 0.0f || dist > min_distance_)
    return getNegUseMask();
  else
    return getPosUseMask();
}

void OccGridUseMaskGenerator::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map)
{
  /// @todo implement model locking
//  if (isLocked())
//    return;

  distance_map_.setMap(occupancy_grid_map);
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::OccGridUseMaskGenerator, l3_footstep_planning::UseMaskGeneratorPlugin)
