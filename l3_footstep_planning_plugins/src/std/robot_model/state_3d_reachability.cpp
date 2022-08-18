#include <l3_footstep_planning_plugins/std/robot_model/state_3d_reachability.h>

namespace l3_footstep_planning
{
State3DReachability::State3DReachability()
  : ReachabilityPlugin("state_3d_reachability")
{}

bool State3DReachability::loadParams(const ParameterSet& params)
{
  if (!ReachabilityPlugin::loadParams(params))
    return false;

  max_step_dz_ = param("max_step_dz", 0.0, true);

  max_state_roll_ = param("max_state_roll", 0.0, true);
  max_state_pitch_ = param("max_state_pitch", 0.0, true);

  max_foothold_roll_ = param("max_foothold_roll", 0.0, true);
  max_foothold_pitch_ = param("max_foothold_pitch", 0.0, true);

  return true;
}

bool State3DReachability::isReachable(const PlanningState& state) const
{
  if (max_step_dz_ > 0.0)
  {
    for (const Step::FootStep::MovingDataPair& p : state.getStep()->footStep().getMovingLinks())
    {
      FootStepData::ConstPtr foot_step = p.second;
      if (std::abs(foot_step->dz) > max_step_dz_)
        return false;
    }
  }

  return isReachable(*state.getState());
}

bool State3DReachability::isReachable(const State& state) const
{
  if (max_state_roll_ > 0.0)
  {
    if (std::abs(state.getFeetCenter().roll()) > max_state_roll_)
      return false;
  }

  if (max_state_pitch_ > 0.0)
  {
    if (std::abs(state.getFeetCenter().pitch()) > max_state_pitch_)
      return false;
  }

  if (max_foothold_roll_ > 0.0 || max_foothold_pitch_ > 0.0)
  {
    for (Foothold::ConstPtr f : state.getFootholds())
    {
      if (!isReachable(*f))
        return false;
    }
  }

  return true;
}

bool State3DReachability::isReachable(const Foothold& foothold) const
{
  if (max_foothold_roll_ > 0.0)
  {
    if (std::abs(foothold.roll()) > max_foothold_roll_)
      return false;
  }

  if (max_foothold_pitch_ > 0.0)
  {
    if (std::abs(foothold.pitch()) > max_foothold_pitch_)
      return false;
  }

  return true;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::State3DReachability, l3_footstep_planning::ReachabilityPlugin)
