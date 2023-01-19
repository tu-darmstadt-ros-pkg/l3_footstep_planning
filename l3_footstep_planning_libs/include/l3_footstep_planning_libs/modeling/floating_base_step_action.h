//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef L3_FOOTSTEP_PLANNING_LIBS_BASE_STEP_ACTION_H__
#define L3_FOOTSTEP_PLANNING_LIBS_BASE_STEP_ACTION_H__

#include <l3_footstep_planning_libs/modeling/discrete_action.h>

namespace l3_footstep_planning
{
/**
 * @brief FloatingBaseStepAction represents the discrete translation. The constructor takes the
 * parameters relative to the base center, but afterwards a FloatingBaseStepAction represents
 * the possible placement relative to the robot body (center).
 */
class FloatingBaseStepAction : public DiscreteAction<FloatingBase>
{
public:
  // typedefs
  typedef l3::SharedPtr<FloatingBaseStepAction> Ptr;
  typedef l3::SharedPtr<const FloatingBaseStepAction> ConstPtr;

  FloatingBaseStepAction(const Pose& neutral_stance, const BaseIndex& base_idx, double dx, double dy, double dpitch, double dyaw, double step_cost, const DiscreteResolution& res);
};

L3_STATIC_ASSERT_MOVEABLE(FloatingBaseStepAction)
}  // namespace l3_footstep_planning

#endif
