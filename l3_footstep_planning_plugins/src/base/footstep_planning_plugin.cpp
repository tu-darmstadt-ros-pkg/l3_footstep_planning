#include <l3_footstep_planning_plugins/base/footstep_planning_plugin.h>

#include <l3_plugins/robot_model.h>

namespace l3_footstep_planning
{
FootstepPlanningPlugin::FootstepPlanningPlugin(const std::string& name)
  : Plugin(name)
{}

bool FootstepPlanningPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!Plugin::loadParams(params))
    return false;

  use_mask_ = param("use_mask", static_cast<unsigned int>(USE_ALWAYS), true);

  // load whitelist
  foot_idx_whitelist_.clear();
  for (const FootIndex& idx : param("foot_idx", FootIndexArray(), true))
    foot_idx_whitelist_.insert(idx);

  // load indirect foot idx list and generate whitelist if not given previously
  bool fill_whitelist = foot_idx_whitelist_.empty();
  indirect_foot_idx_.clear();
  for (const FootInfoPair& p : RobotModel::description()->getFootInfoMap())
  {
    if (p.second.indirect)
      indirect_foot_idx_.insert(p.second.idx);
    else if (fill_whitelist)
      foot_idx_whitelist_.insert(p.second.idx);
  }

  forward_search_ = params.param("forward_search", true);

  return true;
}
}  // namespace l3_footstep_planning
