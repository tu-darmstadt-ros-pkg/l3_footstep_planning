#include <l3_footstep_planning_plugins/aggregator/heuristic.h>

#include <l3_footstep_planning_plugins/aggregator/use_mask_generator.h>

namespace l3_footstep_planning
{
Heuristic::Heuristic()
  : ExtendedPluginAggregator<Heuristic, HeuristicPlugin>("Heuristic")
{}

double Heuristic::getHeuristicValue(const State& from, const State& to, const State& start, const State& goal) const
{
  double h = 0.0;

  // get use masks including superimposition weights
  std::list<UseMaskSuperimposition> masks;
  UseMask mask = UseMaskGenerator::instance().determineHeuristicUseMask(from, to, start, goal, masks);
  masks.push_front(UseMaskSuperimposition(mask));

  for (HeuristicPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);

    // check use mask
    for (const UseMaskSuperimposition& m : masks)
    {
      ROS_ASSERT(m.weight > 0.0 && m.weight <= 1.0);
      if (plugin->canUse(m.use_mask))
      {
        double new_h = m.weight * plugin->getWeight() * plugin->getHeuristicValue(from, to, start, goal);
        //ROS_INFO("H: %s -> %.4f", plugin->getName().c_str(), new_h);
        h += new_h;
        break; // breaking due to assumption of sorted weights (see UseMaskGenerator::mergeSuperimposition)
      }
    }

    ROS_ASSERT(!std::isnan(h));
    ROS_ASSERT(!std::isinf(h));
  }

  //ROS_INFO("----- H: %.4f", h);

  return h;
}
}  // namespace l3_footstep_planning
