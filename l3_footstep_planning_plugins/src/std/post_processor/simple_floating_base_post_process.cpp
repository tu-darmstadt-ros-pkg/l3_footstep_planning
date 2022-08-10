#include <l3_footstep_planning_plugins/std/post_processor/simple_floating_base_post_process.h>

#include <l3_plugins/robot_model.h>
#include <l3_footstep_planning_libs/modeling/state_space_manager.h>

namespace l3_footstep_planning
{
SimpleFloatingBasePostProcess::SimpleFloatingBasePostProcess()
  : PostProcessPlugin("simple_floating_base_post_process")
{}

bool SimpleFloatingBasePostProcess::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PostProcessPlugin::loadParams(params))
    return false;

  res_ = DiscreteResolution(params.getSubset("resolution"));
  stabilize_base_ = param("stabilize_base", true, true);
  post_process_state_ = param("post_process_state", false, true);
  post_process_step_plan_ = param("post_process_step_plan", true, true);

  return true;
}

bool SimpleFloatingBasePostProcess::postProcess(State& state, const FootIndexArray& updated_ids) const
{
  // if updated_ids is empty, it's not called from the planning process
  /// @todo only adding floating base to the goal state but not to intermediate steps will not let the A* converge
  if (post_process_state_ /*|| (updated_ids.empty() && !state.hasFloatingBase())*/)
    state.updateFloatingBase(StateSpaceManager::addFloatingBase(determineFloatingBase(state.getFootholds(), state.getFeetCenter())));

  return true;
}

bool SimpleFloatingBasePostProcess::postProcess(StepPlan& step_plan) const
{
  if (post_process_step_plan_)
  {
    // post process step plan
    for (StepQueue::Entry& e : step_plan.getSteps())
    {
      Step::Ptr step = e.second;
      FootholdConstPtrArray footholds = step->getFootholds();

      FloatingBase::Ptr floating_base = determineFloatingBase(footholds, RobotModel::calcFeetCenter(footholds));

      if (floating_base)
        step->updateMovingFloatingBase(BaseStepData::make(floating_base, floating_base)); /// @todo determine origin floating base
    }
  }

  return true;
}

FloatingBase::Ptr SimpleFloatingBasePostProcess::determineFloatingBase(const FootholdConstPtrArray& footholds, const Pose& feet_center) const
{
  if (footholds.empty())
  {
    ROS_WARN("[%s] Receveived empty footholds!", getName().c_str());
    return FloatingBase::Ptr();
  }

  Pose center = feet_center;

  // set roll and pitch to zero for stabilized base
  if (stabilize_base_)
  {
    center.setRoll(0.0);
    center.setPitch(0.0);
  }

  // compute desired floating base pose
  Pose base_pose = center * RobotModel::kinematics()->calcFeetCenterToBase(*RobotModel::description(), center, footholds);

  // generate discretized floating base
  return makeShared<FloatingBase>(BaseInfo::MAIN_BODY_IDX, base_pose, footholds.front()->header);
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::SimpleFloatingBasePostProcess, l3_footstep_planning::PostProcessPlugin)
