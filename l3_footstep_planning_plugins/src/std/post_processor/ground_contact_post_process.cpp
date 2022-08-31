#include <l3_footstep_planning_plugins/std/post_processor/ground_contact_post_process.h>

#include <l3_footstep_planning_plugins/aggregator/world_model.h>

#include <l3_plugins/robot_model.h>

namespace l3_footstep_planning
{
GroundContactPostProcess::GroundContactPostProcess()
  : PostProcessPlugin("ground_contact_post_processor")
{}

bool GroundContactPostProcess::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PostProcessPlugin::loadParams(params))
    return false;

  getParam("min_sampling_steps_x", min_sampling_steps_x_, 6u);
  getParam("min_sampling_steps_y", min_sampling_steps_y_, 4u);
  getParam("max_sampling_steps_x", max_sampling_steps_x_, 32u);
  getParam("max_sampling_steps_y", max_sampling_steps_y_, 24u);
  getParam("max_intrusion_z", max_intrusion_z_, 0.02);
  getParam("max_ground_clearance", max_ground_clearance_, 0.02);
  getParam("minimal_support", min_contact_support_, 0.9);

  intrusion_norm_factor_ = 1.0 / max_intrusion_z_;
  ground_clearance_norm_factor_ = 1.0 / max_ground_clearance_;

  foot_size_map_.clear();
  for (const FootInfoPair& p : RobotModel::description()->getFootInfoMap())
    foot_size_map_.emplace(p.first, p.second.size);

  return true;
}

bool GroundContactPostProcess::postProcess(Foothold& foothold) const
{
  double support;
  Vector4Array checked_positions;
  bool result = getFootContactSupport(foothold, support, checked_positions);

  foothold.data["gc_support"] = std::move(support);
  foothold.data["gc_checked_positions"] = std::move(checked_positions);

  return result;
}

bool GroundContactPostProcess::getFootContactSupport(const Foothold& foothold, double& support, Vector4Array& checked_positions) const
{
  if (!getFootContactSupport(foothold, support, min_sampling_steps_x_, min_sampling_steps_y_, checked_positions))
    return false;

  // refinement of solution if needed
  if (support == 0.0)  // collision, no refinement
  {
    return true;
  }
  else if (support < 0.95)
  {
    if (!getFootContactSupport(foothold, support, max_sampling_steps_x_, max_sampling_steps_y_, checked_positions))
      return false;
  }

  return true;
}

bool GroundContactPostProcess::getFootContactSupport(const Foothold& foothold, double& support, unsigned int num_sampling_steps_x, unsigned int num_sampling_steps_y,
                                                     Vector4Array& checked_positions) const
{
  support = 0.0;
  checked_positions.clear();

  if (!WorldModel::instance().isTerrainModelAvailable())
  {
    support = 1.0;
    return true;
  }

  unsigned int contacts = 0;
  unsigned int unknown = 0;
  unsigned int total = 0;

  Point orig_pos;
  orig_pos.z() = 0.0;

  // get foot size
  std::map<FootIndex, Vector3>::const_iterator itr = foot_size_map_.find(foothold.idx);
  ROS_ASSERT(itr != foot_size_map_.end());
  const Vector3& foot_size = itr->second;

  double foot_size_half_x = 0.5 * foot_size.x();
  double foot_size_half_y = 0.5 * foot_size.y();

  double sampling_step_x = foot_size.x() / static_cast<double>(num_sampling_steps_x);
  double sampling_step_y = foot_size.y() / static_cast<double>(num_sampling_steps_y);

  for (double y = -foot_size_half_y; y <= foot_size_half_y; y += sampling_step_y)
  {
    orig_pos.setY(y);
    for (double x = -foot_size_half_x; x <= foot_size_half_x; x += sampling_step_x)
    {
      total++;

      // determine point in world frame and get height at this point
      orig_pos.setX(x);

      Point trans_pos = foothold.pose() * orig_pos;

      double height = 0.0;
      if (WorldModel::instance().getHeight(trans_pos.x(), trans_pos.y(), height) == TerrainResult::NO_HEIGHT)
      {
        // ROS_WARN_THROTTLE(1.0, "getFootSupportArea: No height data found at %f/%f", p.getOrigin().getX(), p.getOrigin().getY());
        unknown++;
        continue;
      }

      // diff heights
      double diff = trans_pos.z() - height;  // < 0: Intrustion | > 0: Overhang

      // save evaluated point for visualization
      Vector4 p_checked;
      p_checked.x() = trans_pos.x();
      p_checked.y() = trans_pos.y();
      p_checked.z() = trans_pos.z();
      p_checked.w() = diff < 0.0 ? std::max(-1.0, intrusion_norm_factor_ * diff) : std::min(1.0, ground_clearance_norm_factor_ * diff);  // normalize to -1..1
      // ROS_INFO("%f %f | %f %f | %f", x, y, p.z, height, diff);
      checked_positions.push_back(std::move(p_checked));

      // check diff in z
      if (diff < -max_intrusion_z_)  // collision -> no support!
        return false;
      else if (diff < max_ground_clearance_)  // ground contact
        contacts++;
    }
  }

  if (unknown == total)
  {
    support = 1.0;
    return true;
  }
  else
  {
    /// @ TODO: refinement (center of pressure)
    support = static_cast<double>(contacts) / static_cast<double>(total);
    return min_contact_support_ <= support;
  }
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::GroundContactPostProcess, l3_footstep_planning::PostProcessPlugin)
