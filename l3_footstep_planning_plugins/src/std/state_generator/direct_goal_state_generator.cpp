#include <l3_footstep_planning_plugins/std/state_generator/direct_goal_state_generator.h>

#include <l3_libs/robot_description/base_info.h>

#include <l3_footstep_planning_libs/modeling/state_space_manager.h>

namespace l3_footstep_planning
{
DirectGoalStateGenerator::DirectGoalStateGenerator()
  : StateGeneratorPlugin("direct_goal_state_generator")
{}

std::list<StateGenResult> DirectGoalStateGenerator::generateNearStateResults(const PlanningState& current, const PlanningState& target,
                                                                             const ExpandStatesIdx& state_expansion_idx) const
{
  StateGenResult result;

  for (const FootIndex& idx : state_expansion_idx.foot_idx)
  {
    // do only consider specific foot ids when given
    if (ignoreFootIdx(idx))
      continue;

    Foothold::ConstPtr f = target.getState()->getFoothold(idx);
    if (f)
      result.footholds.push_back(makeShared<Foothold>(*f));
  }

  /// @todo do not ignore state_expansion_idx; but this requires the default state generators to signal explicit expanding
  if (target.getState()->hasFloatingBases())
    result.floating_base = makeShared<FloatingBase>(*target.getState()->getFloatingBases()[l3::BaseInfo::MAIN_BODY_IDX]);

  return std::list<StateGenResult>{ result };
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::DirectGoalStateGenerator, l3_footstep_planning::StateGeneratorPlugin)
