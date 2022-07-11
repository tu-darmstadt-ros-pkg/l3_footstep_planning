#include <l3_footstep_planning_libs/modeling/floating_base_step.h>

namespace l3_footstep_planning
{
FloatingBaseStep::FloatingBaseStep(const Pose& neutral_stance, const BaseIndex& base_idx, double dx, double dy, double dpitch, double dyaw, double step_cost,
                                   const DiscreteResolution& res)
  : DiscreteStep(neutral_stance, base_idx, dx, dy, 0.0, dpitch, dyaw, step_cost, res)
{}
}  // namespace l3_footstep_planning
