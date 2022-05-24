#include <l3_footstep_planning_plugins/base/use_mask_generator_plugin.h>

namespace l3_footstep_planning
{
UseMaskGeneratorPlugin::UseMaskGeneratorPlugin(const std::string& name)
  : Plugin(name)
{}

bool UseMaskGeneratorPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!Plugin::loadParams(params))
    return false;

  pos_use_mask_ = param("pos_use_mask", static_cast<unsigned int>(NO_USE), true);
  neg_use_mask_ = param("neg_use_mask", static_cast<unsigned int>(NO_USE), true);

  return true;
}
}  // namespace l3_footstep_planning
