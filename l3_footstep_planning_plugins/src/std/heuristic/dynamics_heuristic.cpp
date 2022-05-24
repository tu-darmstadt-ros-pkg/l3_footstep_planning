#include <l3_footstep_planning_plugins/std/heuristic/dynamics_heuristic.h>

namespace l3_footstep_planning
{
DynamicsHeuristic::DynamicsHeuristic()
  : HeuristicPlugin("dynamics_heuristic")
{}

bool DynamicsHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::loadParams(params))
    return false;

  getParam("max_body_acc", max_body_acc_, 0.0);
  return true;
}

double DynamicsHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  if (max_body_acc_ <= 0.0)
    return 0.0;

  ROS_ERROR_ONCE("DynamicsHeuristiPlugin not implemented yet.");
  // check if we can still stop in time
  //  const geometry_msgs::Vector3& v = from.getBodyVelocity();
  //  double d_min = (v.x*v.x + v.y*v.y + v.z*v.z)/(2.0*max_body_acc_);

  //  // if planner has issues finding a solution, increase the scaling of d_min or decrease cell size
  //  if (d_min*1.2 > euclidean_distance(from.getX(), from.getY(), from.getZ(), to.getX(), to.getY(), to.getZ()))
  //    return max_heuristic_value_;

  return 0.0;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::DynamicsHeuristic, l3_footstep_planning::HeuristicPlugin)
