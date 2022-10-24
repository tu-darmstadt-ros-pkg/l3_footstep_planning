#include <l3_footstep_planning_plugins/std/heuristic/euclidean_heuristic.h>

namespace l3_footstep_planning
{
EuclideanHeuristic::EuclideanHeuristic()
  : HeuristicPlugin("euclidean_heuristic")
{}

bool EuclideanHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::loadParams(params))
    return false;

  getParam("use_feet_center", use_feet_center_, false, true);

  return true;
}

double EuclideanHeuristic::getHeuristicValue(const State& from, const State& to, const State& start, const State& goal) const
{
  if (use_feet_center_)
  {
    if (from == to)
      return 0.0;

    const Pose& from_pose = from.getFeetCenter();
    const Pose& to_pose = to.getFeetCenter();

    return euclideanDistance(from_pose.x(), from_pose.y(), from_pose.z(), to_pose.x(), to_pose.y(), to_pose.z());
  }
  else
    return normalizeResult(to, HeuristicPlugin::getHeuristicValue(from, to, start, goal));
}

double EuclideanHeuristic::getHeuristicValue(const Foothold& from, const Foothold& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  return euclideanDistance(from.x(), from.y(), from.z(), to.x(), to.y(), to.z());
}

double EuclideanHeuristic::getHeuristicValue(const FloatingBase& from, const FloatingBase& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  return euclideanDistance(from.x(), from.y(), from.z(), to.x(), to.y(), to.z());
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::EuclideanHeuristic, l3_footstep_planning::HeuristicPlugin)
