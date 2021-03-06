#include <l3_footstep_planning_plugins/std/terrain_model/basic_terrain_model.h>

namespace l3_footstep_planning
{
BasicTerrainModel::BasicTerrainModel(const std::string& name)
  : TerrainModelPlugin(name)
{}

bool BasicTerrainModel::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!TerrainModelPlugin::initialize(params))
    return false;

  // subscribe
  std::string topic = param("topic", std::string("terrain_model"), true);
  terrain_model_sub_ = nh_.subscribe(topic, 1, &BasicTerrainModel::setTerrainModel, this);

  return true;
}

void BasicTerrainModel::reset()
{
  TerrainModelPlugin::reset();

  if (terrain_model_)
    terrain_model_->reset();
}

bool BasicTerrainModel::isTerrainModelAvailable() const { return terrain_model_ && terrain_model_->hasTerrainModel(); }

void BasicTerrainModel::setTerrainModel(const l3_terrain_modeling::TerrainModelMsg::ConstPtr& terrain_model)
{
  if (isLocked())
    return;

  UniqueLock lock = uniqueLockModel();

  // update terrain model
  if (!terrain_model_)
    terrain_model_.reset(new l3_terrain_modeling::TerrainModel(*terrain_model));
  else
    terrain_model_->fromMsg(*terrain_model);

  this->notifyTerrainModelUpdate();
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::BasicTerrainModel, l3_footstep_planning::TerrainModelPlugin)
