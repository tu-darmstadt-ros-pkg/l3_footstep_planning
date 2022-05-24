#include <l3_footstep_planning_plugins/base/world_model_plugin.h>

namespace l3_footstep_planning
{
WorldModelPlugin::WorldModelPlugin(const std::string& name)
  : FootstepPlanningPlugin(name)
{}

bool WorldModelPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!FootstepPlanningPlugin::loadParams(params))
    return false;

  unsigned int collision_check_mask;
  params.getParam("collision_check/collision_check_mask", (int&)collision_check_mask);
  getParam("collision_check_flag", (int&)collision_check_flag_, -1, true);
  collision_check_enabled_ = this->collision_check_flag_ & collision_check_mask;

  return true;
}

bool WorldModelPlugin::isLocked() const
{
  UniqueLock lock(model_lock_, boost::try_to_lock);
  return !lock.owns_lock();
}

bool WorldModelPlugin::isCollisionCheckAvailable() const { return collision_check_enabled_; }

bool WorldModelPlugin::isAccessible(const State& state) const
{
  for (Foothold::ConstPtr f : state.getFootholds())
  {
    if (!isAccessible(*f))
      return false;
  }

  return true;
}

bool WorldModelPlugin::isAccessible(const PlanningState& state) const { return isAccessible(*state.getState()); }
}  // namespace l3_footstep_planning
