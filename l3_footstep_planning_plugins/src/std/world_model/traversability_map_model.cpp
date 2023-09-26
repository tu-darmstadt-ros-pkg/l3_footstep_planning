#include <l3_footstep_planning_plugins/std/world_model/traversability_map_model.h>

namespace l3_footstep_planning
{
TraversabilityMapModel::TraversabilityMapModel()
  : WorldModelPlugin("traversability_map_model")
{}

bool TraversabilityMapModel::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!WorldModelPlugin::loadParams(params))
    return false;

  // get the parameters
  getParam("traversability_map_topic", traversability_map_topic_, std::string("/traversability_estimation/traversability_map"));
  getParam("traversability_map_layer", traversability_map_layer_, std::string("traversability"));
  getParam("min_traversability_weight", min_traversability_weight_, 0.5);
  getParam("mean_traversability_weight", mean_traversability_weight_, 0.5);
  getParam("traversability_threshold", traversability_threshold_, 0.75);
  getParam("unknown_traversability_value", unknown_traversability_value_, 1.0f, true);
  getParam("check_foothold_accessibility", check_foothold_accessibility_, true, true);
  getParam("check_floating_base_accessibility", check_floating_base_accessibility_, true, true);
  getParam("upper_body_margin_x", upper_body_margin_x_, 0.0, true);
  getParam("upper_body_margin_y", upper_body_margin_y_, 0.0, true);
  getParam("foothold_margin_x", foothold_margin_x_, 0.0, true);
  getParam("foothold_margin_y", foothold_margin_y_, 0.0, true);
  getParam("num_sampling_points_x", num_sampling_points_x_, 5, true);
  getParam("num_sampling_points_y", num_sampling_points_y_, 10, true);

  if (min_traversability_weight_ + mean_traversability_weight_ != 1.0)
  {
    ROS_ERROR("[%s]: min_traversability_weight_ + mean_traversability_weight_ must be 1.0!", getName().c_str());
    return false;
  }

  if (min_traversability_weight_ < 0.0 || min_traversability_weight_ > 1.0 || mean_traversability_weight_ < 0.0 || mean_traversability_weight_ > 1.0)
  {
    ROS_ERROR("[%s]: The weights must be in [0.0, 1.0]!", getName().c_str());
    return false;
  }

  if (traversability_threshold_ < 0.0 || traversability_threshold_ > 1.0)
  {
    ROS_ERROR("[%s]: traversability_threshold_ must be in [0.0, 1.0]!", getName().c_str());
    return false;
  }

  // if the min traversability is so low that the following inequation holds, the accessibility check will be false and the loop can be exited early:
  // current_min_traversability * min_traversability_weight_ + mean_traversability_weight_ * 1 < traversability_threshold_
  // => current_min_traversability < (traversability_threshold_ - mean_traversability_weight_) / min_traversability_weight_
  // since we already assume the best value for the mean traversability (1.0)
  early_exit_threshold_ = (traversability_threshold_ - mean_traversability_weight_) / min_traversability_weight_;

  if (unknown_traversability_value_ < 0.0 || unknown_traversability_value_ > 1.0)
  {
    ROS_ERROR("[%s]: unknown_traversability_value_ must be in [0.0, 1.0]!", getName().c_str());
    return false;
  }

  unknown_area_is_accessible_ = unknown_traversability_value_ >= traversability_threshold_;

  if (num_sampling_points_x_ < 2 || num_sampling_points_y_ < 2)
  {
    ROS_ERROR("[%s]: num_sampling_points_x_ and num_sampling_points_y_ must at least be 2!", getName().c_str());
    return false;
  }

  // get the size of the upper body
  BaseInfo base_info;
  RobotModel::description()->getBaseInfo(BaseInfo::MAIN_BODY_IDX, base_info);
  upper_body_size_ = Vector2(base_info.size.x() + upper_body_margin_x_, base_info.size.y() + upper_body_margin_y_);

  // get the sizes of the feet
  std::vector<FootIndex> foot_idx_list = RobotModel::description()->footIdxList();
  feet_sizes_.resize(foot_idx_list.size());
  feet_shapes_.resize(foot_idx_list.size());
  for (const FootIndex& footIndex : foot_idx_list)
  {
    FootInfo foot_info = RobotModel::description()->getFootInfo(footIndex);
    size_t index = RobotModel::description()->getFootIdx(footIndex);
    feet_sizes_[index] = Vector2(foot_info.size.x() + foothold_margin_x_, foot_info.size.y() + foothold_margin_y_);
    feet_shapes_[index] = foot_info.shape;
  }

  return true;
}

bool TraversabilityMapModel::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!WorldModelPlugin::initialize(params))
    return false;

  // subscribe to the traversability map
  traversability_map_sub_ = nh_.subscribe(traversability_map_topic_, 1, &TraversabilityMapModel::mapCallback, this);

  return true;
}

bool TraversabilityMapModel::isAccessible(const Foothold& foothold) const
{
  // skip check if disabled
  if (!check_foothold_accessibility_)
    return true;

  if (!traversability_map_.exists(traversability_map_layer_))
  {
    ROS_ERROR_ONCE("[%s]: Traversability map layer '%s' does not exist!", getName().c_str(), traversability_map_layer_.c_str());
    return false;
  }

  Vector2 foot_size = feet_sizes_[RobotModel::description()->getFootIdx(foothold.idx)];
  FootInfo::Shape foot_shape = feet_shapes_[RobotModel::description()->getFootIdx(foothold.idx)];

  if (foot_shape == FootInfo::Shape::CUBOID)
    return isPolygonAccessible(Vector2(foothold.x(), foothold.y()), foot_size, foothold.yaw());

  if (foot_shape == FootInfo::Shape::SPHERICAL)
  {
    l3::Position2D pos(foothold.x(), foothold.y());

    if (traversability_map_.isInside(pos))
    {
      float traversability = traversability_map_.atPosition(traversability_map_layer_, pos);
      if (!std::isnan(traversability))
        return traversability >= traversability_threshold_;
    }
    return unknown_area_is_accessible_;
  }

  ROS_ERROR_ONCE("[%s]: Foot shape not supported!", getName().c_str());
  return false;
}

bool TraversabilityMapModel::isAccessible(const FloatingBase& floatingBase) const
{
  // skip check if disabled
  if (!check_floating_base_accessibility_)
    return true;

  if (traversability_map_.exists(traversability_map_layer_))
    return isPolygonAccessible(Vector2(floatingBase.x(), floatingBase.y()), upper_body_size_, floatingBase.yaw());

  ROS_ERROR_ONCE("[%s]: Traversability map layer '%s' does not exist!", getName().c_str(), traversability_map_layer_.c_str());
  return false;
}

void TraversabilityMapModel::mapCallback(const grid_map_msgs::GridMapConstPtr& traversability_map_new)
{
  if (isLocked())
    return;

  grid_map::GridMapRosConverter::fromMessage(*traversability_map_new, traversability_map_);
}

bool TraversabilityMapModel::isPolygonAccessible(const Vector2& position, const Vector2& size, double yaw) const
{
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  double half_length = size.x() * 0.5;
  double half_width = size.y() * 0.5;

  // get the step size in the length direction, as well as the step vectors corresponding to the yaw angle
  double length_step = size.x() / (num_sampling_points_x_ - 1);
  double length_step_x = length_step * cos_yaw;
  double length_step_y = length_step * sin_yaw;

  // get the step size in the width direction, as well as the step vectors corresponding to the yaw angle
  double width_step = size.y() / (num_sampling_points_y_ - 1);
  double width_step_x = width_step * sin_yaw;
  double width_step_y = -width_step * cos_yaw;

  // calculate the bottom left corner of the polygon considering the yaw angle
  l3::Position2D bottom_left(position.x() - half_length * cos_yaw - half_width * sin_yaw, position.y() - half_length * sin_yaw + half_width * cos_yaw);

  // iterate over the polygon with the smaller number of points in the outer loop
  if (num_sampling_points_x_ >= num_sampling_points_y_)
    return iteratePolygon(num_sampling_points_y_, num_sampling_points_x_, bottom_left, width_step_x, width_step_y, length_step_x, length_step_y);

  return iteratePolygon(num_sampling_points_x_, num_sampling_points_y_, bottom_left, length_step_x, length_step_y, width_step_x, width_step_y);
}

bool TraversabilityMapModel::iteratePolygon(int num_sampling_points_min, int num_sampling_points_max, l3::Position2D& bottom_left_corner, double min_step_x, double min_step_y,
                                            double max_step_x, double max_step_y) const
{
  // initialize the count variables
  int num_cells = 0;
  double sum_traversability = 0.0;
  double min_traversability = 1.0;

  // iterate over the polygon
  for (int i = 0; i < num_sampling_points_min; ++i)
  {
    // subtracted max_step_x and max_step_y because of the pos update at the start of the loop
    l3::Position2D pos = bottom_left_corner + l3::Position2D(i * min_step_x - max_step_x, i * min_step_y - max_step_y);
    for (int j = 0; j < num_sampling_points_max; ++j)
    {
      ++num_cells;
      pos += l3::Position2D(max_step_x, max_step_y);

      if (!traversability_map_.isInside(pos))
      {
        sum_traversability += unknown_traversability_value_;
        continue;
      }

      float traversability = traversability_map_.atPosition(traversability_map_layer_, pos);
      if (std::isnan(traversability))
      {
        sum_traversability += unknown_traversability_value_;
        continue;
      }

      sum_traversability += traversability;
      if (traversability >= min_traversability)
        continue;

      min_traversability = traversability;
      // early exit if the minimum traversability is below the early exit threshold
      if (min_traversability < early_exit_threshold_)
        return false;
    }
  }

  // Use the weights, minimum traversability, and mean traversability to determine if the polygon is accessible
  return min_traversability * min_traversability_weight_ + sum_traversability / num_cells * mean_traversability_weight_ >= traversability_threshold_;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::TraversabilityMapModel, l3_footstep_planning::WorldModelPlugin)
