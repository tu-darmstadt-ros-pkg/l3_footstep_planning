#include <l3_footstep_planning_plugins/base/heuristic_plugin.h>

namespace l3_footstep_planning
{
HeuristicPlugin::HeuristicPlugin(const std::string& name)
  : FootstepPlanningPlugin(name)
{}

bool HeuristicPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!FootstepPlanningPlugin::loadParams(params))
    return false;

  weight_ = param("weight", 1.0, true);

  max_heuristic_value_ = params.param("max_heuristic_value", std::numeric_limits<double>::max(), true);

  return true;
}

double HeuristicPlugin::getHeuristicValue(const State& from, const State& to, const State& start, const State& goal) const
{
  double h_val = 0.0;

  for (Foothold::ConstPtr fh_from : from.getFootholds())
  {
    ROS_ASSERT(f_from);

    // do only consider specific foot ids when given
    if (ignoreFootIdx(fh_from->idx))
      continue;

    Foothold::ConstPtr f_to = to.getFoothold(fh_from->idx);
    if (f_to)
      h_val += getHeuristicValue(*fh_from, *f_to, start, goal);
  }

  for(FloatingBase::ConstPtr fb_from : from.getFloatingBases())
  {
    ROS_ASSERT(f_from);

    FloatingBase::ConstPtr fb_to = to.getFloatingBase(fb_from->idx);
    if(fb_to)
      h_val += getHeuristicValue(*fb_from, *fb_to, start, goal);
  }

  return h_val;
}

double HeuristicPlugin::normalizeResult(const State& state, double h_val) const
{
  unsigned int num_feet = footIdxWhitelist().empty() ? state.getFootholds().size() : applyFootIdxWhitelist(state.getFootholds()).size();
  unsigned int num_bases = state.getFloatingBases().size();

  // normalize result
  if ((num_feet + num_bases) > 1)
    h_val /= static_cast<double>(num_feet + num_bases);

  return h_val;
}
}  // namespace l3_footstep_planning
