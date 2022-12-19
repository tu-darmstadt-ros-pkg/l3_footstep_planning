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
  std::string terrain_topic = param("terrain_topic", std::string("/terrain_model"), true);
  terrain_model_sub_ = nh_.subscribe(terrain_topic, 1, &BasicTerrainModel::setTerrainModel, this);
  std::string elevation_map_topic = param("elevation_map_topic", std::string("/elevation_map"), true);
  grid_map_sub_ = nh_.subscribe(elevation_map_topic, 1, &BasicTerrainModel::setGridMap, this);

  // advertise
  terrain_model_pub_ = nh_.advertise<grid_map_msgs::GridMap>(terrain_topic + "_grid_map", 1, true);

  return true;
}

void BasicTerrainModel::reset()
{
  TerrainModelPlugin::reset();

  if (terrain_model_)
    terrain_model_->reset();
}

bool BasicTerrainModel::isTerrainModelAvailable() const { return terrain_model_ && terrain_model_->hasTerrainModel(); }

void BasicTerrainModel::setTerrainModel(l3_terrain_modeling::TerrainModelMsg::ConstPtr terrain_model)
{
  if (isLocked())
    return;

  UniqueLock lock = uniqueLockModel();

  // update terrain model
  if (!terrain_model_)
    terrain_model_ = l3::makeShared<l3_terrain_modeling::TerrainModel>(*terrain_model);
  else
    terrain_model_->fromMsg(*terrain_model);

  // republish received map
  if (terrain_model_pub_.getNumSubscribers() > 0)
    terrain_model_pub_.publish(terrain_model->map);

  this->notifyTerrainModelUpdate();
}

void BasicTerrainModel::setGridMap(grid_map_msgs::GridMap::ConstPtr grid_map)
{
  if (isLocked())
    return;

  UniqueLock lock = uniqueLockModel();

  // update terrain model
  if (!terrain_model_)
    terrain_model_ = l3::makeShared<l3_terrain_modeling::TerrainModel>(*grid_map);
  else
    terrain_model_->fromMsg(*grid_map);

  // republish received map
  if (terrain_model_pub_.getNumSubscribers() > 0)
    terrain_model_pub_.publish(grid_map);

  this->notifyTerrainModelUpdate();
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::BasicTerrainModel, l3_footstep_planning::TerrainModelPlugin)
