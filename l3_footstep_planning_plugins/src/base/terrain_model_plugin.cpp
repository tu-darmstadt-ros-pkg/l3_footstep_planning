#include <l3_footstep_planning_plugins/base/terrain_model_plugin.h>

#include <l3_plugins/robot_model.h>

namespace l3_footstep_planning
{
TerrainModelPlugin::TerrainModelPlugin(const std::string& name)
  : Plugin(name)
{}

bool TerrainModelPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!Plugin::loadParams(params))
    return false;

  sample_max_height_under_foot_ = param("sample_max_height_under_foot", false, true);
  sampling_steps_x_ = param("sampling_steps_x", 2u, true);
  sampling_steps_y_ = param("sampling_steps_y", 2u, true);

  foot_size_map_.clear();
  for (const FootInfoPair& p : RobotModel::description()->getFootInfoMap())
    foot_size_map_.emplace(p.first, p.second.size);

  return true;
}

bool TerrainModelPlugin::isLocked() const
{
  UniqueLock lock(model_lock_, boost::try_to_lock);
  return !lock.owns_lock();
}

TerrainResult TerrainModelPlugin::update3DData(Foothold& foothold) const
{
  if (sample_max_height_under_foot_)
  {
    double height;
    if (getMaxHeightUnderFoot(foothold, height) == TerrainResult::OK)
    {
      foothold.setZ(height);
      return TerrainResult::OK;
    }
    else
      return TerrainResult::NO_DATA;
  }
  else
    return update3DDataImpl(foothold);
}

TerrainResult TerrainModelPlugin::getMaxHeightUnderFoot(const Foothold& foothold, double& height) const
{
  TerrainResult result = TerrainResult::NO_DATA;
  height = -std::numeric_limits<double>::max();

  if (!isTerrainModelAvailable())
    return result;

  Point orig_pos;
  orig_pos.z() = 0.0;

  // get foot size
  std::map<FootIndex, Vector3>::const_iterator itr = foot_size_map_.find(foothold.idx);
  ROS_ASSERT(itr != foot_size_map_.end());
  const Vector3& foot_size = itr->second;

  double foot_size_half_x = 0.5 * foot_size.x();
  double foot_size_half_y = 0.5 * foot_size.y();

  double sampling_step_x = foot_size.x() / static_cast<double>(sampling_steps_x_);
  double sampling_step_y = foot_size.y() / static_cast<double>(sampling_steps_y_);

  for (double y = -foot_size_half_y; y <= foot_size_half_y; y += sampling_step_y)
  {
    orig_pos.setY(y);
    for (double x = -foot_size_half_x; x <= foot_size_half_x; x += sampling_step_x)
    {
      // determine point in world frame and get height at this point
      orig_pos.setX(x);

      Point trans_pos = foothold.pose() * orig_pos;

      double h = 0.0;
      if (getHeight(trans_pos.x(), trans_pos.y(), h) != TerrainResult::NO_HEIGHT)
      {
        height = std::max(height, h);
        result = TerrainResult::OK;
      }
    }
  }

  return result;
}
}  // namespace l3_footstep_planning
