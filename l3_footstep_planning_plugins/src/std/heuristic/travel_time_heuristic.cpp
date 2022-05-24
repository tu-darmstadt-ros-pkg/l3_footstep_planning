#include <l3_footstep_planning_plugins/std/heuristic/travel_time_heuristic.h>

namespace l3_footstep_planning
{
TravelTimeHeuristic::TravelTimeHeuristic()
  : HeuristicPlugin("travel_time_heuristic")
{}

bool TravelTimeHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::loadParams(params))
    return false;

  getParam("use_feet_center", use_feet_center_, false, true);

  if (getParam("linear_vel", linear_vel_inv_, 0.0, true))
     linear_vel_inv_ = 1.0 / linear_vel_inv_;

  if (getParam("angular_vel", angular_vel_inv_, 0.0, true))
     angular_vel_inv_ = 1.0 / angular_vel_inv_;

  return true;
}

double TravelTimeHeuristic::getHeuristicValue(const State& from, const State& to, const State& start, const State& goal) const
{
  if (from == to)
    return 0.0;

  if (use_feet_center_)
  {
    const Pose& pose_from = from.getFeetCenter();
    const Pose& pose_to = to.getFeetCenter();

    double linear_cost = linear_vel_inv_ > 0.0 ? euclidean_distance(pose_from.x(), pose_from.y(), pose_from.z(), pose_to.x(), pose_to.y(), pose_to.z()) * linear_vel_inv_ : 0.0;
    double angular_cost = angular_vel_inv_ > 0.0 ? std::abs(shortestAngularDistance(pose_from.yaw(), pose_to.yaw())) * angular_vel_inv_ : 0.0;

    return linear_cost + angular_cost;
  }
  else
  {
    double h_val = 0.0;

    for (Foothold::ConstPtr f_from : from.getFootholds())
    {
      ROS_ASSERT(f_from);

      // do only consider specific foot ids when given
      if (ignoreFootIdx(f_from->idx))
        continue;

      // find largest time cost
      Foothold::ConstPtr f_to = to.getFoothold(f_from->idx);
      if (f_to)
        h_val = std::max(h_val, getHeuristicValue(*f_from, *f_to, start, goal));
    }

    return h_val;
  }
}

double TravelTimeHeuristic::getHeuristicValue(const Foothold& from, const Foothold& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  double linear_cost = linear_vel_inv_ > 0.0 ? euclidean_distance(from.x(), from.y(), from.z(), to.x(), to.y(), to.z()) * linear_vel_inv_ : 0.0;
  double angular_cost = angular_vel_inv_ > 0.0 ? std::abs(shortestAngularDistance(from.yaw(), to.yaw())) * angular_vel_inv_ : 0.0;

  return linear_cost + angular_cost;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::TravelTimeHeuristic, l3_footstep_planning::HeuristicPlugin)
