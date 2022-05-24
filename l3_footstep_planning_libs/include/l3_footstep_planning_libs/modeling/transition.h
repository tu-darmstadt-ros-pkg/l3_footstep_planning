//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_TRANSITION_H__
#define L3_FOOTSTEP_PLANNING_TRANSITION_H__

#include <list>

#include <l3_footstep_planning_libs/modeling/state.h>
#include <l3_footstep_planning_libs/modeling/footstep.h>

#include <l3_footstep_planning_libs/modeling/state_id.h>

namespace l3_footstep_planning
{
class Transition
{
public:
  // typedefs
  typedef l3::SharedPtr<Transition> Ptr;
  typedef l3::SharedPtr<const Transition> ConstPtr;

  Transition(Footstep::ConstPtr footstep, StateHashed::ConstPtr predecessor, StateHashed::ConstPtr successor, double cost, double risk)
    : footstep_(footstep)
    , predecessor_(predecessor)
    , successor_(successor)
    , cost_(cost)
    , risk_(risk)
  {}

  inline Footstep::ConstPtr getFootstep() const { return footstep_; }

  inline StateHashed::ConstPtr getPredecessor() const { return predecessor_; }
  inline StateHashed::ConstPtr getSuccessor() const { return successor_; }

  inline double getCost() const { return cost_; }
  inline double getRisk() const { return risk_; }

protected:
  Footstep::ConstPtr footstep_;

  StateHashed::ConstPtr predecessor_;
  StateHashed::ConstPtr successor_;

  double cost_;
  double risk_;
};

typedef std::list<Transition> TransitionList;
typedef std::list<Transition::ConstPtr> TransitionConstPtrList;

L3_STATIC_ASSERT_MOVEABLE(Transition)
}  // namespace l3_footstep_planning

#endif
