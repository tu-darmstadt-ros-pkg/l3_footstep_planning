#include <l3_footstep_planning_plugins/aggregator/reachability.h>

namespace l3_footstep_planning
{
Reachability::Reachability()
  : ExtendedPluginAggregator<Reachability, ReachabilityPlugin>("Reachability")
{}

bool Reachability::isReachable(const PlanningState& state) const
{
  for (ReachabilityPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);

    if (!plugin->isReachable(state))
      return false;
  }
  return true;
}

bool Reachability::isReachable(const State& state) const
{
  for (ReachabilityPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);

    if (!plugin->isReachable(state))
      return false;
  }
  return true;
}
}  // namespace l3_footstep_planning
