#include <l3_footstep_planning_plugins/std/step_cost_estimator/ground_contact_step_cost_estimator.h>

#include <angles/angles.h>

namespace l3_footstep_planning
{
GroundContactStepCostEstimator::GroundContactStepCostEstimator()
  : StepCostEstimatorPlugin("ground_contact_step_cost_estimator")
{}

bool GroundContactStepCostEstimator::getCost(const PlanningState& state, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  cost = 0.0;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;

  double min_support = 1.0;

  for (const Step::StepDataPair& p : state.getStep()->getStepDataMap())
  {
    FootStepData::ConstPtr foot_step = p.second;

    Foothold::ConstPtr foothold = forwardSearch() ? foot_step->target : foot_step->origin;

    // do only consider specific foot ids when given
    if (ignoreFootIdx(foothold->idx))
      continue;

    double support;
    if (foothold->data.get("gc_support", support))
      min_support = std::min(min_support, support);
    else
    {
      //ROS_WARN_ONCE("[GroundContactStepCostEstimator] getCost (State UID: %lu): Step did not contain any 'gc_support' data. Please check your plugin setup. This warning will be printed only once.", state.getState()->getUID());
      return true;
    }
  }

  if (min_support > 0.0)
  {
    if (min_support < 1.0)
      cost_multiplier = risk_multiplier = 1.0 / min_support;
    return true;
  }

  return false;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::GroundContactStepCostEstimator, l3_footstep_planning::StepCostEstimatorPlugin)
