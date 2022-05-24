#include <l3_footstep_planning_plugins/std/heuristic/step_cost_heuristic.h>

#include <l3_footstep_planning_plugins/std/step_range_polygon.h>

namespace l3_footstep_planning
{
StepCostHeuristic::StepCostHeuristic()
  : HeuristicPlugin("step_cost_heuristic")
{}

bool StepCostHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::loadParams(params))
    return false;

  getParam("step_cost", step_cost_, 0.0, true);
  getParam("diff_angle_cost", diff_angle_cost_, 0.0, true);

  if (getParam("max_step_dist_x", max_step_dist_x_inv_, 0.0, true))
    max_step_dist_x_inv_ = 1.0 / max_step_dist_x_inv_;
  if (getParam("max_step_dist_y", max_step_dist_y_inv_, 0.0, true))
    max_step_dist_y_inv_ = 1.0 / max_step_dist_y_inv_;

  return true;
}

/// @todo: Handle case when robot can move multiple foot in parallel (mean or max time?)
double StepCostHeuristic::getHeuristicValue(const Foothold& from, const Foothold& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  double expected_steps = 0;

  // expected steps
  Transform dstep = Foothold::getDelta2D(from, to);

  if (step_cost_ > 0.0)
  {
    double expected_steps_x = std::abs(dstep.x()) * max_step_dist_x_inv_;
    double expected_steps_y = std::abs(dstep.y()) * max_step_dist_y_inv_;
    expected_steps = norm(expected_steps_x, expected_steps_y);

    // at least one additional full step is required if yaw is not matching
    if (std::abs(dstep.yaw()) > 0.0)
      expected_steps = std::max(expected_steps, 1.0);
  }

  double diff_angle = 0.0;
  if (diff_angle_cost_ > 0.0)
    diff_angle = std::abs(dstep.yaw());

  // ROS_WARN("-------------------------------");
  // ROS_INFO("x: %f %f %f", dstep.x(), max_step_dist_x_, std::abs(dstep.x()) / max_step_dist_x_);
  // ROS_INFO("y: %f %f %f", dstep.y(), max_step_dist_y_, std::abs(dstep.y()) / max_step_dist_y_);
  // ROS_INFO("steps: %f, dist: %f, cost: %f", expected_steps, dist, (dist + expected_steps * step_cost_ + diff_angle * diff_angle_cost_));

  return expected_steps * step_cost_ + diff_angle * diff_angle_cost_;
}

double StepCostHeuristic::getHeuristicValue(const FloatingBase& from, const FloatingBase& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  double expected_steps = 0;

  // expected steps
  Transform dstep = FloatingBase::getDelta2D(from, to);

  if (step_cost_ > 0.0)
  {
    double expected_steps_x = std::abs(dstep.x()) * max_step_dist_x_inv_;
    double expected_steps_y = std::abs(dstep.y()) * max_step_dist_y_inv_;
    expected_steps = norm(expected_steps_x, expected_steps_y);

    // at least one additional full step is required if yaw is not matching
    if (std::abs(dstep.yaw()) > 0.0)
      expected_steps = std::max(expected_steps, 1.0);
  }

  double diff_angle = 0.0;
  if (diff_angle_cost_ > 0.0)
    diff_angle = std::abs(dstep.yaw());

  return expected_steps * step_cost_ + diff_angle * diff_angle_cost_;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::StepCostHeuristic, l3_footstep_planning::HeuristicPlugin)
