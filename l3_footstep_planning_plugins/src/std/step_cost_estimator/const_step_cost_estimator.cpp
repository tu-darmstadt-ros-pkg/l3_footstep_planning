#include <l3_footstep_planning_plugins/std/step_cost_estimator/const_step_cost_estimator.h>

namespace l3_footstep_planning
{
ConstStepCostEstimator::ConstStepCostEstimator()
  : StepCostEstimatorPlugin("const_step_cost_estimator")
{}

bool ConstStepCostEstimator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!StepCostEstimatorPlugin::loadParams(params))
    return false;

  getParam("step_cost", const_step_cost, 0.1);

  return true;
}

bool ConstStepCostEstimator::getCost(const PlanningState& /*state*/, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  cost = const_step_cost;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;
  return true;
}

bool ConstStepCostEstimator::getCost(const FootStepData& /*step_data*/, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  cost = const_step_cost;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;
  return true;
}

bool ConstStepCostEstimator::getCost(const BaseStepData& /*base_step_data*/, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  cost = const_step_cost;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;
  return true;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::ConstStepCostEstimator, l3_footstep_planning::StepCostEstimatorPlugin)
