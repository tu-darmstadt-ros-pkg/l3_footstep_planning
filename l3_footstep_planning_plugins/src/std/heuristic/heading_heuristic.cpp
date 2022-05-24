#include <l3_footstep_planning_plugins/std/heuristic/heading_heuristic.h>

namespace l3_footstep_planning
{
HeadingHeuristic::HeadingHeuristic()
  : HeuristicPlugin("heading_heuristic")
{}

bool HeadingHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::loadParams(params))
    return false;

  max_start_dist_sq_ = param("max_start_dist", 0.0, true);
  max_start_dist_sq_ *= max_start_dist_sq_;
  max_goal_dist_sq_ = param("max_goal_dist", 0.0, true);
  max_goal_dist_sq_ *= max_goal_dist_sq_;

  start_dyaw_cost_scale_ = param("start_dyaw_cost_scale", 0.0, true);
  travel_dyaw_cost_scale_ = param("travel_dyaw_cost_scale", 0.0, true);
  goal_dyaw_cost_scale_ = param("goal_dyaw_cost_scale", 0.0, true);

  can_walk_backwards_ = param("can_walk_backwards", true, true);

  return true;
}

double HeadingHeuristic::getHeuristicValue(const State& from, const State& /*to*/, const State& start, const State& goal) const
{
  const Pose& current = from.getFeetCenter();

  // 1. check goal heading
  if (goal_dyaw_cost_scale_ > 0.0)
  {
    const Pose& goal_pose = goal.getFeetCenter();
    double yaw = goal_pose.yaw();

    if (max_goal_dist_sq_ > 0.0)
    {
      double goal_dist_sq = (current.getPosition() - goal_pose.getPosition()).squaredNorm();
      if (goal_dist_sq < max_goal_dist_sq_)
      {
        yaw = lerpAngle(yaw, current.yaw(), 1.0, goal_dist_sq / max_goal_dist_sq_);
        return std::abs(shortestAngularDistance(current.yaw(), yaw)) * goal_dyaw_cost_scale_;
      }
    }
    else
      return std::abs(shortestAngularDistance(current.yaw(), yaw)) * goal_dyaw_cost_scale_;
  }

  // 2. check start heading
  if (start_dyaw_cost_scale_ > 0.0)
  {
    const Pose& start_pose = start.getFeetCenter();
    double yaw = start_pose.yaw();

    if (max_start_dist_sq_ > 0.0)
    {
      double start_dist_sq = (current.getPosition() - start_pose.getPosition()).squaredNorm();
      if (start_dist_sq < max_start_dist_sq_)
      {
        yaw = lerpAngle(yaw, current.yaw(), 1.0, start_dist_sq / max_goal_dist_sq_);
        return std::abs(shortestAngularDistance(current.yaw(), yaw)) * start_dyaw_cost_scale_;
      }
    }
    else
      return std::abs(shortestAngularDistance(current.yaw(), yaw)) * start_dyaw_cost_scale_;
  }

  // 3. check travel heading
  if (travel_dyaw_cost_scale_ > 0.0)
  {
    const Pose& goal_pose = goal.getFeetCenter();
    double yaw = calcHeading(current, goal_pose);
    double dyaw = std::abs(shortestAngularDistance(current.yaw(), yaw));

    if (can_walk_backwards_)
    {
      // check final goal heading (keep orientation of goal)
      double goal_travel_dyaw = std::abs(shortestAngularDistance(yaw, goal_pose.yaw()));
      if (goal_travel_dyaw > M_PI_2)
        dyaw = std::abs(shortestAngularDistance(current.yaw() + M_PI, yaw));
    }

    return dyaw * travel_dyaw_cost_scale_;
  }

  return 0.0;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::HeadingHeuristic, l3_footstep_planning::HeuristicPlugin)
