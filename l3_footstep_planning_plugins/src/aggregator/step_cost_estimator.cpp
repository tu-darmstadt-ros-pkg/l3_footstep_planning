#include <l3_footstep_planning_plugins/aggregator/step_cost_estimator.h>

#include <l3_footstep_planning_plugins/aggregator/use_mask_generator.h>

namespace l3_footstep_planning
{
StepCostEstimator::StepCostEstimator()
  : ExtendedPluginAggregator<StepCostEstimator, StepCostEstimatorPlugin>("StepCostEstimator")
{}

bool StepCostEstimator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!ExtendedPluginAggregator<StepCostEstimator, StepCostEstimatorPlugin>::loadParams(params))
    return false;

  params.getParam("max_risk", max_risk_, 1.0);

  return true;
}

bool StepCostEstimator::getCost(const PlanningState& state, const State& start, const State& goal, double& cost, double& risk) const
{
  cost = 0.0;
  double cost_multiplier = 1.0;
  risk = 0.0;
  double risk_multiplier = 1.0;

  // get use masks including superimposition weights
  std::list<UseMaskSuperimposition> masks;
  UseMask mask = UseMaskGenerator::instance().determineStepCostEstimatorUseMask(state, start, goal, masks);
  masks.push_front(UseMaskSuperimposition(mask));

  // ROS_INFO("-----------------------------------");
  for (StepCostEstimatorPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);

    // check use mask
    for (const UseMaskSuperimposition& m : masks)
    {
      ROS_ASSERT(m.weight > 0.0 && m.weight <= 1.0);
      if (plugin->canUse(m.use_mask))
      {
        double c, c_m, r, r_m;
        if (!plugin->getCost(state, c, c_m, r, r_m))
          return false;

        //ROS_INFO("C: %s -> %.4f / %.4f", plugin->getName().c_str(), m.weight * c, m.weight * r);

        ROS_ASSERT(!std::isnan(c) && !std::isinf(c));
        ROS_ASSERT(!std::isnan(r) && !std::isinf(r));
        ROS_ASSERT(c >= 0.0);
        ROS_ASSERT(cost_multiplier >= 1.0);
        ROS_ASSERT(risk_multiplier >= 1.0);

        cost += m.weight * c;
        cost_multiplier *= c_m;
        risk += m.weight * r;
        risk_multiplier *= r_m;
        // ROS_INFO("[%s]: %.3f %.3f %.3f %.3f", step_cost_estimator->getName().c_str(), c, r, c_m, r_m);

        break; // breaking due to assumption of sorted weights (see UseMaskGenerator::mergeSuperimposition)
      }
    }
  }

  //ROS_INFO("----- C: %.4f / %.4f", cost, risk);

  cost *= cost_multiplier;
  risk *= risk_multiplier;

  ROS_ASSERT(cost > 0.0);  // g-values must increase!

  return risk < max_risk_;
}

bool StepCostEstimator::getCost(const PlanningState& state, const State& start, const State& goal, float& cost, float& risk) const
{
  double cost_d, risk_d;
  bool result = getCost(state, start, goal, cost_d, risk_d);
  cost = static_cast<float>(cost_d);
  risk = static_cast<float>(risk_d);
  return result;
}
}  // namespace l3_footstep_planning
