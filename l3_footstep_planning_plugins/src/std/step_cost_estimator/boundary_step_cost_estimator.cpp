#include <l3_footstep_planning_plugins/std/step_cost_estimator/boundary_step_cost_estimator.h>

#include <angles/angles.h>

namespace l3_footstep_planning
{
BoundaryStepCostEstimator::BoundaryStepCostEstimator()
  : StepCostEstimatorPlugin("boundary_step_cost_estimator")
{}

bool BoundaryStepCostEstimator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!StepCostEstimatorPlugin::loadParams(params))
    return false;

  params.getParam("boundary_step_cost_estimator/max_diff_z", max_diff_z);
  params.getParam("boundary_step_cost_estimator/long_step_dist", long_step_dist);
  params.getParam("boundary_step_cost_estimator/min_yaw_seperation_enlargement", min_yaw_seperation_enlargement);
  params.getParam("boundary_step_cost_estimator/yaw_enlarged_min_seperation", yaw_enlarged_min_seperation);
  params.getParam("boundary_step_cost_estimator/cost_roll_abs", cost_roll_abs);
  params.getParam("boundary_step_cost_estimator/cost_pitch_abs", cost_pitch_abs);
  params.getParam("boundary_step_cost_estimator/cost_yaw_rel", cost_yaw_rel);
  params.getParam("boundary_step_cost_estimator/cost_height_diff_rel", cost_height_diff_rel);

  return true;
}

bool BoundaryStepCostEstimator::getCost(const FootStepData& step_data, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  risk = 0.0;

  double diff_z = std::abs(step_data.dz);

  // add costs
  if (diff_z >= max_diff_z)
    return false;

  // all forward long steps should be more expensive
  if (step_data.dx > long_step_dist)
    risk += step_data.dx - long_step_dist;

  // if foot is turned step more outside
  if (std::abs(step_data.dyaw) >= min_yaw_seperation_enlargement && std::abs(step_data.dy) <= yaw_enlarged_min_seperation)
    return false;

  // determine risk
  risk += cost_roll_abs * std::abs(step_data.target->roll());
  risk += cost_pitch_abs * std::abs(step_data.target->pitch());
  // risk += cost_yaw_rel * std::abs(turn_rate_diff);
  risk += cost_height_diff_rel * diff_z;

  cost = risk * risk;
  cost_multiplier = 1.0;
  risk_multiplier = 1.0;

  return true;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::BoundaryStepCostEstimator, l3_footstep_planning::StepCostEstimatorPlugin)
