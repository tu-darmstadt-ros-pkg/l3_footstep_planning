#include <l3_footstep_planning_vis_tools/base/planning_vis_plugin.h>

#include <l3_plugins/robot_model.h>

namespace l3_footstep_planning
{
PlanningVisPlugin::PlanningVisPlugin(const std::string& name)
  : Plugin(name)
{}

bool PlanningVisPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!Plugin::loadParams(params))
    return false;

  // load whitelist
  foot_idx_whitelist_.clear();
  for (const FootIndex& idx : param("foot_idx", FootIndexArray(), true))
    foot_idx_whitelist_.insert(idx);

  // generate whitelist from indirect foot idx if not given previously
  if (foot_idx_whitelist_.empty())
  {
    for (const FootInfoPair& p : RobotModel::description()->getFootInfoMap())
    {
      if (!p.second.indirect)
        foot_idx_whitelist_.insert(p.second.idx);
    }
  }

  return true;
}

void PlanningVisPlugin::visualize(const StepPlan& step_plan)
{
  for (const StepQueue::Entry e : step_plan.getSteps())
    visualize(*e.second);
}
}  // namespace l3_footstep_planning
