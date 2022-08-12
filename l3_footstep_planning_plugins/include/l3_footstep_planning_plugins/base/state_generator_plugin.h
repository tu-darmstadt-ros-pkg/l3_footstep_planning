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

#ifndef L3_FOOTSTEP_PLANNING_STATE_GENERATOR_PLUGIN_H__
#define L3_FOOTSTEP_PLANNING_STATE_GENERATOR_PLUGIN_H__

#include <ros/ros.h>

#include <l3_footstep_planning_plugins/base/footstep_planning_plugin.h>

#include <l3_footstep_planning_libs/modeling/planning_state.h>

namespace l3_footstep_planning
{
using namespace vigir_generic_params;

/// @todo extend to multiple bases planning
struct StateGenResult
{
  StateGenResult()
    : footholds()
    , floating_base()
  {}

  StateGenResult(const FootholdPtrArray& footholds, FloatingBase::Ptr floating_base = FloatingBase::Ptr())
    : footholds(footholds)
    , floating_base(floating_base)
  {}

  StateGenResult(FootholdPtrArray&& footholds, FloatingBase::Ptr&& floating_base = FloatingBase::Ptr())
    : footholds(std::move(footholds))
    , floating_base(std::move(floating_base))
  {}

  FootholdPtrArray footholds;
  FloatingBase::Ptr floating_base;
};

class StateGeneratorPlugin : public virtual FootstepPlanningPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<StateGeneratorPlugin> Ptr;
  typedef l3::SharedPtr<const StateGeneratorPlugin> ConstPtr;

  StateGeneratorPlugin(const std::string& name);

  bool isUnique() const final { return false; }

  /**
   * @brief Generates list of state result arrays representing a possible predecessor state.
   * However, each foothold array should only contain the footholds of moved legs.
   * @param state Current state from which all valid preceding foothold arrays and floating bases should be derived from
   * @param start Start state of planning request
   * @param goal Goal state of planning request
   * @param state_expansion_idx ids of feet and floating bases to move
   * @return List of list of all predecessor state results
   */
  virtual std::list<StateGenResult> generatePredStateResults(const PlanningState& state, const State& start, const State& goal, const ExpandStatesIdx& state_expansion_idx) const
  {
    return std::list<StateGenResult>();
  }

  /**
   * @brief Generates list of state result arrays representing a possible successor state.
   * However, each foothold array should only contain the footholds of moved legs.
   * @param state Current state from which all valid succeeding foothold arrays and floating bases should be derived from
   * @param start Start state of planning request
   * @param goal Goal state of planning request
   * @param state_expansion_idx ids of feet and floating bases to move
   * @return List of list of all successor state results
   */
  virtual std::list<StateGenResult> generateSuccStateResults(const PlanningState& state, const State& start, const State& goal, const ExpandStatesIdx& state_expansion_idx) const
  {
    return std::list<StateGenResult>();
  }

  /**
   * @brief Generates list of state result arrays representing a transition into a target state. When the target
   * cannot be reached within a single step directly, the resulting footholds and bases should at least provide a best effort
   * solution to get closer to the target state in order to simplify the search for the next iteration.
   * The resulting state may be invalid, which is then sorted out by following processing steps.
   * @param current Current state
   * @param target Target state
   * @param state_expansion_idx ids of feet and bases to move
   * @return List of list of new state results
   */
  virtual std::list<StateGenResult> generateNearStateResults(const PlanningState& current, const PlanningState& target, const ExpandStatesIdx& state_expansion_idx) const
  {
    return std::list<StateGenResult>();
  }
};
}  // namespace l3_footstep_planning

#endif
