#include <l3_footstep_planning_plugins/std/step_cost_estimator/euclidean_step_cost_estimator.h>

namespace l3_footstep_planning
{
EuclideanStepCostEstimator::EuclideanStepCostEstimator()
  : StepCostEstimatorPlugin("euclidean_step_cost_estimator")
{}

bool EuclideanStepCostEstimator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!StepCostEstimatorPlugin::loadParams(params))
    return false;

  getParam("use_feet_center", use_feet_center_, false, true);

  return true;
}

bool EuclideanStepCostEstimator::getCost(const PlanningState& state, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  if (use_feet_center_)
  {
    const Pose& from = state.getAdjacentState()->getFeetCenter();
    const Pose& to = state.getState()->getFeetCenter();

    cost = euclidean_distance(from.x(), from.y(), from.z(), to.x(), to.y(), to.z());
    cost_multiplier = 1.0;
    risk = 0.0;
    risk_multiplier = 1.0;

    return true;
  }
  else if (StepCostEstimatorPlugin::getCost(state, cost, cost_multiplier, risk, risk_multiplier))
  {
    normalizeResult(state, cost, risk);
    return true;
  }
  else
    return false;
}

bool EuclideanStepCostEstimator::getCost(const FootStepData& step_data, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  cost = norm(step_data.dx, step_data.dy);
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;
  return true;
}

bool EuclideanStepCostEstimator::getCost(const BaseStepData& base_step_data, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  cost = norm(base_step_data.dx, base_step_data.dy);
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;
  return true;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::EuclideanStepCostEstimator, l3_footstep_planning::StepCostEstimatorPlugin)
