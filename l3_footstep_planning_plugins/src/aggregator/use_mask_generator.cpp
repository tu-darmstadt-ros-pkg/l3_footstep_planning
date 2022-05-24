#include <l3_footstep_planning_plugins/aggregator/use_mask_generator.h>

namespace l3_footstep_planning
{
UseMaskGenerator::UseMaskGenerator()
  : ExtendedPluginAggregator<UseMaskGenerator, UseMaskGeneratorPlugin>("UseMaskGenerator")
{}


UseMask UseMaskGenerator::determineStateGenerationUseMask(const PlanningState& state, const State& start, const State& goal) const
{
  UseMask mask = USE_ALWAYS;

  for (UseMaskGeneratorPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);
    mask |= plugin->determineStateGenerationUseMask(state, start, goal);
  }

  return mask;
}

UseMask UseMaskGenerator::determineStepCostEstimatorUseMask(const PlanningState& state, const State& start, const State& goal, std::list<UseMaskSuperimposition>& masks) const
{
  masks.clear();
  UseMask mask = USE_ALWAYS;

  for (UseMaskGeneratorPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);
    std::list<UseMaskSuperimposition> new_masks;
    mask |= plugin->determineStepCostEstimatorUseMask(state, start, goal, new_masks);
    mergeSuperimposition(masks, new_masks);
  }

  return mask;
}

UseMask UseMaskGenerator::determineHeuristicUseMask(const State& from, const State& to, const State& start, const State& goal, std::list<UseMaskSuperimposition>& masks) const
{
  masks.clear();
  UseMask mask = USE_ALWAYS;

  for (UseMaskGeneratorPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);
    std::list<UseMaskSuperimposition> new_masks;
    mask |= plugin->determineHeuristicUseMask(from, to, start, goal, new_masks);
    mergeSuperimposition(masks, new_masks);
  }

  return mask;
}

void UseMaskGenerator::mergeSuperimposition(std::list<UseMaskSuperimposition>& out, const std::list<UseMaskSuperimposition>& in) const
{
  for (const UseMaskSuperimposition& m_in : in)
  {
    bool merged = false;

    for (UseMaskSuperimposition& m_out : out)
    {
      if (m_out.use_mask == m_in.use_mask)
      {
        m_out.weight = std::max(m_out.weight, m_in.weight);
        merged = true;
        break;
      }
    }

    if (!merged)
      out.push_back(m_in);
  }
}
}  // namespace l3_footstep_planning
