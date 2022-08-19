#include <l3_footstep_planning_plugins/std/step_cost_estimator/travel_time_step_cost_estimator.h>

namespace l3_footstep_planning
{
TravelTimeStepCostEstimator::TravelTimeStepCostEstimator()
  : StepCostEstimatorPlugin("travel_time_step_cost_estimator")
{}

bool TravelTimeStepCostEstimator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!StepCostEstimatorPlugin::loadParams(params))
    return false;

  getParam("use_feet_center", use_feet_center_, false, true);

  if (getParam("linear_vel", linear_vel_inv_, 0.0, true))
    linear_vel_inv_ = 1.0 / linear_vel_inv_;

  if (getParam("angular_vel", angular_vel_inv_, 0.0, true))
    angular_vel_inv_ = 1.0 / angular_vel_inv_;

  return true;
}

bool TravelTimeStepCostEstimator::getCost(const PlanningState& state, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  if (use_feet_center_)
  {
    const Pose& from = state.getAdjacentState()->getFeetCenter();
    const Pose& to = state.getState()->getFeetCenter();

    double linear_cost = linear_vel_inv_ > 0.0 ? euclidean_distance(from.x(), from.y(), from.z(), to.x(), to.y(), to.z()) * linear_vel_inv_ : 0.0;
    double angular_cost = angular_vel_inv_ > 0.0 ? std::abs(shortestAngularDistance(from.yaw(), to.yaw())) * angular_vel_inv_ : 0.0;

    cost = linear_cost + angular_cost;
    cost_multiplier = 1.0;
    risk = 0.0;
    risk_multiplier = 1.0;
  }
  else
  {
    for (const Step::FootStep::MovingDataPair& p : state.getStep()->footStep().getMovingLinks())
    {
      FootStepData::ConstPtr foot_step = p.second;
      ROS_ASSERT(foot_step);

      // do only consider specific foot ids when given
      if (ignoreFootIdx(foot_step->target->idx))
        continue;

      double c, c_m, r, r_m;
      if (!getCost(*foot_step, c, c_m, r, r_m))
        return false;

      ROS_ASSERT(c_m >= 1.0);
      ROS_ASSERT(r_m >= 1.0);

      // take max cost over all steps
      if (c > cost)
      {
        cost = c;
        cost_multiplier = c_m;
        risk = r;
        risk_multiplier = r_m;
      }
    }
  }

  return true;
}

bool TravelTimeStepCostEstimator::getCost(const FootStepData& step_data, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  double linear_cost = linear_vel_inv_ > 0.0 ? norm(step_data.dx, step_data.dy, step_data.dz) * linear_vel_inv_ : 0.0;
  double angular_cost = angular_vel_inv_ > 0.0 ? std::abs(step_data.dyaw) * angular_vel_inv_ : 0.0;

  cost = linear_cost + angular_cost;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;

  return true;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::TravelTimeStepCostEstimator, l3_footstep_planning::StepCostEstimatorPlugin)
