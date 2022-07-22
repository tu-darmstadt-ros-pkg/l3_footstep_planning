#include <l3_footstep_planning_plugins/std/state_generator/direct_goal_state_generator.h>

#include <l3_libs/robot_description/base_info.h>

namespace l3_footstep_planning
{
DirectGoalStateGenerator::DirectGoalStateGenerator()
  : StateGeneratorPlugin("direct_goal_state_generator")
{}

bool DirectGoalStateGenerator::loadParams(const ParameterSet& params)
{
  getParam("strict_mode", strict_mode_, false, true);

  return true;
}

std::list<StateGenResult> DirectGoalStateGenerator::generateNearStateResults(const PlanningState& current, const PlanningState& target,
                                                                             const ExpandStatesIdx& state_expansion_idx) const
{
  StateGenResult result;

  // in strict mode do only generate a state when at least one foothold or floating base is in goal
  if (strict_mode_ && !hasEqualFootholdElement(current.getState(), target.getState()) && !hasEqualFloatingBaseElement(current.getState(), target.getState()))
    return std::list<StateGenResult>();

  for (const FootIndex& idx : state_expansion_idx.foot_idx)
  {
    // do only consider specific foot ids when given
    if (ignoreFootIdx(idx))
      continue;

    Foothold::ConstPtr f = target.getState()->getFoothold(idx);
    if (f)
      result.footholds.push_back(makeShared<Foothold>(*f));
  }

  /// @todo Not sure about ignore state_expansion_idx for floating bases;
  /// but usually it requires to expand the floating base to reach the final goal configuration
  if (target.getState()->hasFloatingBases())
    result.floating_base = makeShared<FloatingBase>(*target.getState()->getFloatingBases()[l3::BaseInfo::MAIN_BODY_IDX]);

  return std::list<StateGenResult>{ result };
}

bool DirectGoalStateGenerator::hasEqualFootholdElement(StateHashed::ConstPtr current, StateHashed::ConstPtr target)
{
  for (FootholdHashed::ConstPtr fh_target : target->getFootholdsHashed())
  {
    FootholdHashed::ConstPtr fh_current = current->getFoothold(fh_target->idx);
    if (fh_current && fh_current->getHashValue() == fh_target->getHashValue())
      return true;
  }
  return false;
}

bool DirectGoalStateGenerator::hasEqualFloatingBaseElement(StateHashed::ConstPtr current, StateHashed::ConstPtr target)
{
  for (FloatingBaseHashed::ConstPtr fb_target : target->getFloatingBasesHashed())
  {
    FloatingBaseHashed::ConstPtr fb_current = current->getFloatingBase(fb_target->idx);
    if (fb_current && fb_current->getHashValue() == fb_target->getHashValue())
      return true;
  }
  return false;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::DirectGoalStateGenerator, l3_footstep_planning::StateGeneratorPlugin)
