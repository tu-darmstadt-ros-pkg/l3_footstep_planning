//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
// Based on http://wiki.ros.org/footstep_planner by Johannes Garimort and Armin Hornung
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

#ifndef L3_FOOTSTEP_PLANNING_STATE_SPACE_H__
#define L3_FOOTSTEP_PLANNING_STATE_SPACE_H__

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <tr1/unordered_set>
#include <tr1/hashtable.h>

#include <sbpl/headers.h>

#include <l3_footstep_planning_libs/modeling/state_space_manager.h>
#include <l3_footstep_planning_libs/modeling/foot_step_action.h>
#include <l3_footstep_planning_libs/modeling/planning_state.h>

#include <l3_footstep_planner/environment_parameters.h>

namespace l3_footstep_planning
{
class StateSpace
{
public:
  // typedefs
  typedef l3::SharedPtr<StateSpace> Ptr;
  typedef l3::SharedPtr<StateSpace> ConstPtr;

  StateSpace(const EnvironmentParameters& params);

  void reset();

  UID updateStart(const State& state);
  UID updateGoal(const State& state);

  inline PlanningStateHashed::ConstPtr getStartPlanningState() const { return start_state_; }
  inline PlanningStateHashed::ConstPtr getGoalPlanningState() const { return goal_state_; }

  inline StateHashed::ConstPtr getStartState() const { return start_state_->getState(); }
  inline StateHashed::ConstPtr getGoalState() const { return goal_state_->getState(); }

  inline void setStartFootIndex(const FootIndex& start_foot_idx) { start_foot_idx_ = start_foot_idx; }
  inline const FootIndex& getStartFootIndex() const { return start_foot_idx_; }

protected:
  const EnvironmentParameters& params_;

  PlanningStateHashed::ConstPtr start_state_;
  PlanningStateHashed::ConstPtr goal_state_;
  FootIndex start_foot_idx_;
};
}  // namespace l3_footstep_planning

#endif
