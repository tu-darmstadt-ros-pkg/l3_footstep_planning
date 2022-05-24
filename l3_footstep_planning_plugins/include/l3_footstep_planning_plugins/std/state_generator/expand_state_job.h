//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_THREADING_EXPAND_STATE_JOB_H__
#define L3_FOOTSTEP_PLANNING_PLUGINS_THREADING_EXPAND_STATE_JOB_H__

#include <ros/ros.h>

#include <l3_footstep_planning_libs/modeling/footstep.h>
#include <l3_footstep_planning_libs/modeling/planning_state.h>

namespace l3_footstep_planning
{
namespace threading
{
struct ExpandStateJob
{
  // typedefs
  typedef l3::SharedPtr<ExpandStateJob> Ptr;
  typedef l3::SharedPtr<const ExpandStateJob> ConstPtr;

  /**
   * @brief Multi-threading worker in order to generate a new planning state
   * @param state Current planning state
   * @param start Start state of planning request
   * @param goal Goal state of planning request
   * @param new_footholds New footholds of new planning state (can be empty)
   * @param new_floating_base New floating base of new planning state (can be null)
   * @param forward True, if planner is in forward planning mode
   * @param lazy When true, some processing steps are skipped (currently StepCostEstimation)
   * @param basic_cost Base cost of the transition on which the step cost is added up
   * @param basic_risk Base risk of the transition on which the step risk is added up
   */
  ExpandStateJob(PlanningState::ConstPtr state, State::ConstPtr start, State::ConstPtr goal, const FootholdPtrArray& new_footholds, FloatingBase::ConstPtr new_floating_base,
                 bool forward, bool lazy = false, double basic_cost = 0.0, double basic_risk = 0.0);
  ExpandStateJob(PlanningState::ConstPtr state, State::ConstPtr start, State::ConstPtr goal, const FootholdHashedConstPtrArray& new_footholds,
                 FloatingBaseHashed::ConstPtr new_floating_base, bool forward, bool lazy = false, double basic_cost = 0.0, double basic_risk = 0.0);

  inline static ExpandStateJob::Ptr make(PlanningState::ConstPtr state, State::ConstPtr start, State::ConstPtr goal, const FootholdPtrArray& new_footholds,
                                         FloatingBase::ConstPtr new_floating_base, bool forward, bool lazy = false, double basic_cost = 0.0, double basic_risk = 0.0)
  {
    return ExpandStateJob::Ptr(new ExpandStateJob(state, start, goal, new_footholds, new_floating_base, forward, lazy, basic_cost, basic_risk));
  }

  inline static ExpandStateJob::Ptr make(PlanningState::ConstPtr state, State::ConstPtr start, State::ConstPtr goal, const FootholdHashedConstPtrArray& new_footholds,
                                         FloatingBaseHashed::ConstPtr new_floating_base, bool forward, bool lazy = false, double basic_cost = 0.0, double basic_risk = 0.0)
  {
    return ExpandStateJob::Ptr(new ExpandStateJob(state, start, goal, new_footholds, new_floating_base, forward, lazy, basic_cost, basic_risk));
  }

  void run();  // called by worker thread

  PlanningStateHashed::ConstPtr next;
  double cost;
  double risk;
  bool successful;

private:
  StateHashed::ConstPtr current_state_;

  State::ConstPtr start_;
  State::ConstPtr goal_;

  FootholdPtrArray new_footholds_;
  FootholdHashedConstPtrArray new_footholds_hashed_;

  FloatingBase::ConstPtr new_floating_base_;
  FloatingBaseHashed::ConstPtr new_floating_base_hashed_;

  bool forward_;
  bool lazy_;
};

L3_STATIC_ASSERT_MOVEABLE(ExpandStateJob)
}  // namespace threading
}  // namespace l3_footstep_planning

#endif
