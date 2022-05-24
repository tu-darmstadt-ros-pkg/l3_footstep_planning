#include <l3_footstep_planning_plugins/base/terrain_model_plugin.h>

namespace l3_footstep_planning
{
TerrainModelPlugin::TerrainModelPlugin(const std::string& name)
  : Plugin(name)
{}

bool TerrainModelPlugin::isLocked() const
{
  UniqueLock lock(model_lock_, boost::try_to_lock);
  return !lock.owns_lock();
}
}  // namespace l3_footstep_planning
