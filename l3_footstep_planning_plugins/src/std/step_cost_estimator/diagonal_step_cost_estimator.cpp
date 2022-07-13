#include <l3_footstep_planning_plugins/std/step_cost_estimator/diagonal_step_cost_estimator.h>

#include <l3_plugins/robot_model.h>

namespace l3_footstep_planning
{
DiagonalStepCostEstimator::DiagonalStepCostEstimator()
  : StepCostEstimatorPlugin("diagonal_step_cost_estimator")
{}

bool DiagonalStepCostEstimator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!StepCostEstimatorPlugin::loadParams(params))
    return false;

  getParam("cost_scale", cost_scale_, 1.0, true);

  DiscreteResolution res = params.getSubset("resolution");
  res_x_ = res.toCont().x;

  return true;
}

bool DiagonalStepCostEstimator::getCost(const PlanningState& state, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  cost = 0.0;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;

  FootholdPtrMap map = l3::footholdArrayToMap<FootholdPtrMap>(RobotModel::getNeutralStance(state.getState()->getFeetCenter()));

  for (Foothold::ConstPtr f : state.getState()->getFootholds())
  {
    FootholdPtrMap::const_iterator itr = map.find(f->idx);
    if (itr == map.end())
      continue;

    Foothold::ConstPtr neutral = itr->second;
    Transform delta = Foothold::getDelta2D(*neutral, *f);

    if (std::abs(delta.x()) > res_x_)
      cost += std::abs(delta.y());
  }

  cost *= cost_scale_;

  return true;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::DiagonalStepCostEstimator, l3_footstep_planning::StepCostEstimatorPlugin)
