#include <l3_footstep_planning_plugins/std/post_processor/step_dynamics_post_process.h>

namespace l3_footstep_planning
{
StepDynamicsPostProcess::StepDynamicsPostProcess()
  : PostProcessPlugin("step_dynamics_post_processor")
{}

bool StepDynamicsPostProcess::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PostProcessPlugin::loadParams(params))
    return false;

  default_swing_height_ = param("swing_height", 0.1, true);
  default_sway_duration_ = param("sway_duration", 0.0, true);
  default_step_duration_ = param("step_duration", 1.0);

  calc_body_vel_ = param("calc_body_vel", false, true);

  return true;
}

bool StepDynamicsPostProcess::postProcess(PlanningState& state) const
{
  if (calc_body_vel_)
  {
    /// @todo Use floating base here if available
    const Pose& cur = state.getState()->getFeetCenter();
    const Pose& adj = state.getAdjacentState()->getFeetCenter();

    state.data["body_vel"] = (adj.getPosition() - cur.getPosition()).norm() / (default_sway_duration_ + default_step_duration_);
  }

  return true;
}

bool StepDynamicsPostProcess::postProcess(Step& step) const
{
  for (Step::FootStep::MovingDataPair& p : step.footStep().getMovingLinks())
  {
    ROS_ASSERT(p.second);
    determineStepAttributes(*p.second);
    determineTimings(*p.second);
  }

  for (Step::BaseStep::MovingDataPair& p : step.baseStep().getMovingLinks())
  {
    ROS_ASSERT(p.second);
    determineStepAttributes(*p.second);
    determineTimings(*p.second);
  }

  return true;
}

void StepDynamicsPostProcess::determineStepAttributes(FootStepData& step) const { step.swing_height = default_swing_height_; }

void StepDynamicsPostProcess::determineTimings(FootStepData& step) const
{
  step.sway_duration = default_sway_duration_;
  step.step_duration = default_step_duration_;
}

void StepDynamicsPostProcess::determineStepAttributes(BaseStepData& step) const {}

void StepDynamicsPostProcess::determineTimings(BaseStepData& step) const { step.step_duration = default_step_duration_; }
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::StepDynamicsPostProcess, l3_footstep_planning::PostProcessPlugin)
