#include <l3_footstep_planning_libs/modeling/foot_step_action.h>

namespace l3_footstep_planning
{
FootStepAction::FootStepAction(const Pose& neutral_stance, const FootIndex& foot_idx, double dx, double dy, double dyaw, double step_cost, const DiscreteResolution& res)
  : DiscreteAction(neutral_stance, foot_idx, dx, dy, 0.0, 0.0, dyaw, step_cost, res)
{}
}  // namespace l3_footstep_planning
