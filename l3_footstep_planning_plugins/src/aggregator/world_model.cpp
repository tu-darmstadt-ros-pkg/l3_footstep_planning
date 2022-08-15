#include <l3_footstep_planning_plugins/aggregator/world_model.h>

namespace l3_footstep_planning
{
WorldModel::WorldModel()
  : ExtendedPluginAggregator<WorldModel, WorldModelPlugin>("WorldModel")
{}

void WorldModel::loadPlugins(bool print_warning)
{
  ExtendedPluginAggregator<WorldModel, WorldModelPlugin>::loadPlugins(print_warning);

  // get terrain model
  vigir_pluginlib::PluginManager::getPlugin(terrain_model_plugin_);
  if (terrain_model_plugin_)
  {
    ROS_INFO("[WorldModel] Found terrain model:");
    ROS_INFO("    %s (%s)", terrain_model_plugin_->getName().c_str(), terrain_model_plugin_->getTypeClass().c_str());
  }
}

bool WorldModel::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!ExtendedPluginAggregator<WorldModel, WorldModelPlugin>::loadParams(params))
    return false;

  bool result = true;

  if (terrain_model_plugin_)
    result &= terrain_model_plugin_->loadParams(params);

  return result;
}

void WorldModel::resetPlugins()
{
  ExtendedPluginAggregator<WorldModel, WorldModelPlugin>::resetPlugins();

  if (terrain_model_plugin_)
    terrain_model_plugin_->reset();
}

WorldModel::ModelLocks WorldModel::lockModel() const
{
  ModelLocks locks;

  for (WorldModelPlugin::Ptr plugin : getPlugins())
    locks.push_back(plugin->lockModel());

  if (terrain_model_plugin_)
    locks.push_back(terrain_model_plugin_->sharedLockModel());

  return locks;
}

bool WorldModel::isAccessible(const Foothold& foothold) const
{
  for (WorldModelPlugin::Ptr plugin : getPlugins())
  {
    if (plugin && plugin->isCollisionCheckAvailable() && !plugin->isAccessible(foothold))
      return false;
  }
  return true;
}

bool WorldModel::isAccessible(const FloatingBase& floating_base) const
{
  for (WorldModelPlugin::Ptr plugin : getPlugins())
  {
    if (plugin && plugin->isCollisionCheckAvailable() && !plugin->isAccessible(floating_base))
      return false;
  }
  return true;
}

bool WorldModel::isAccessible(const State& state) const
{
  for (WorldModelPlugin::Ptr plugin : getPlugins())
  {
    if (plugin && plugin->isCollisionCheckAvailable() && !plugin->isAccessible(state))
      return false;
  }
  return true;
}

bool WorldModel::isAccessible(const PlanningState& state) const
{
  for (WorldModelPlugin::Ptr plugin : getPlugins())
  {
    if (plugin && plugin->isCollisionCheckAvailable() && !plugin->isAccessible(state))
      return false;
  }
  return true;
}

void WorldModel::useTerrainModel(bool enabled) { use_terrain_model_ = enabled; }

bool WorldModel::isTerrainModelAvailable() const { return terrain_model_plugin_ && terrain_model_plugin_->isTerrainModelAvailable(); }

TerrainModelPlugin::ConstPtr WorldModel::getTerrainModel() const { return terrain_model_plugin_; }

TerrainResult WorldModel::getHeight(double x, double y, double& height) const
{
  if (!use_terrain_model_ || !isTerrainModelAvailable())
    return TerrainResult::OK;

  return terrain_model_plugin_->getHeight(x, y, height);
}

TerrainResult WorldModel::update3DData(Foothold& foothold) const
{
  if (!use_terrain_model_ || !isTerrainModelAvailable())
    return TerrainResult::OK;

  return terrain_model_plugin_->update3DData(foothold);
}
}  // namespace l3_footstep_planning
