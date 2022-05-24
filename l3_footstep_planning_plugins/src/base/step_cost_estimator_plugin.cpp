#include <l3_footstep_planning_plugins/base/step_cost_estimator_plugin.h>

namespace l3_footstep_planning
{
StepCostEstimatorPlugin::StepCostEstimatorPlugin(const std::string& name)
  : FootstepPlanningPlugin(name)
{}

bool StepCostEstimatorPlugin::getCost(const PlanningState& state, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  cost = 0.0;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;

  for (const Step::StepDataPair& p : state.getStep()->getStepDataMap())
  {
    StepData::ConstPtr step_data = p.second;
    ROS_ASSERT(step_data);

    // do only consider specific foot ids when given
    if (ignoreFootIdx(step_data->target->idx))
      continue;

    double c, c_m, r, r_m;
    if (!getCost(*step_data, c, c_m, r, r_m))
      return false;

    ROS_ASSERT(c_m >= 1.0);
    ROS_ASSERT(r_m >= 1.0);

    cost += c;
    cost_multiplier *= c_m;
    risk += r;
    risk_multiplier *= r_m;
  }

  for (const Step::BaseStepDataPair& p : state.getStep()->getMovingFloatingBaseMap())
  {
    BaseStepData::ConstPtr base_step_data = p.second;
    ROS_ASSERT(step_data);

    double c, c_m, r, r_m;
    if (!getCost(*base_step_data, c, c_m, r, r_m))
      return false;

    ROS_ASSERT(c_m >= 1.0);
    ROS_ASSERT(r_m >= 1.0);

    cost += c;
    cost_multiplier *= c_m;
    risk += r;
    risk_multiplier *= r_m;
  }

  return true;
}

bool StepCostEstimatorPlugin::getCost(const StepData& /*step_data*/, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  // nothing to do here
  cost = 0.0;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;
  return true;
}

bool StepCostEstimatorPlugin::getCost(const BaseStepData& /*base_step_data*/, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  // nothing to do here
  cost = 0.0;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;
  return true;
}

void StepCostEstimatorPlugin::normalizeResult(const PlanningState& state, double& cost, double& risk) const
{
  unsigned int num_feet = footIdxWhitelist().empty() ? state.getState()->getFootholds().size() : applyFootIdxWhitelist(state.getState()->getFootholds()).size();

  // normalize result
  if (num_feet > 1)
  {
    cost /= static_cast<double>(num_feet);
    risk /= static_cast<double>(num_feet);
  }
}
}  // namespace l3_footstep_planning
