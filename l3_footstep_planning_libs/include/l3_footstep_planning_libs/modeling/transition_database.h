//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_TRANSITION_DATABASE_H__
#define L3_FOOTSTEP_PLANNING_TRANSITION_DATABASE_H__

#include <unordered_map>
#include <unordered_set>

#include <l3_footstep_planning_libs/typedefs.h>
#include <l3_footstep_planning_libs/hash.h>

#include <l3_footstep_planning_libs/modeling/transition.h>
#include <l3_footstep_planning_libs/modeling/transition_id.h>

namespace l3_footstep_planning
{
using namespace l3;

typedef std::unordered_set<TransitionHashed::ConstPtr, TransitionHasher> TransitionSet;

class TransitionDataBase
{
private:
  typedef std::unordered_map<TransitionID, TransitionHashed::ConstPtr, TransitionHasher> TransitionMap;
  typedef std::unordered_map<StateID, TransitionSet, StateHasher> TransitionListMap;

public:
  // typedefs
  typedef l3::SharedPtr<TransitionDataBase> Ptr;
  typedef l3::SharedPtr<const TransitionDataBase> ConstPtr;

  TransitionDataBase();

  inline TransitionHashed::ConstPtr operator()(const StateID& pred, const StateID& succ) const { return get(pred, succ); }
  inline TransitionHashed::ConstPtr operator()(const TransitionID& id) const { return get(id); }

  inline TransitionHashed::ConstPtr operator[](const TransitionID& id) const { return get(id); }

  size_t size() const;
  bool empty() const;
  void clear();

  TransitionHashed::ConstPtr insert(const Transition& transition);
  TransitionHashed::ConstPtr insert(Transition&& transition);

  inline bool has(const StateID& pred, const StateID& succ) const { return has(TransitionID(pred, succ)); }
  bool has(const TransitionID& id) const;

  inline TransitionHashed::ConstPtr get(const StateID& pred, const StateID& succ) const { return get(TransitionID(pred, succ)); }
  TransitionHashed::ConstPtr get(const TransitionID& id) const;

  const TransitionSet& getPredecessors(const StateID& id) const;
  const TransitionSet& getSuccessors(const StateID& id) const;

  inline void remove(const StateID& pred, const StateID& succ) { remove(TransitionID(pred, succ)); }
  void remove(const TransitionID& id);

  inline unsigned long hits() const { return hits_; }
  inline unsigned long miss() const { return miss_; }

protected:
  mutable Mutex mutex_;

  TransitionMap map_;
  TransitionListMap pred_map_;  // maps a state to a set of all its predecessor
  TransitionListMap succ_map_;  // maps a state to a set of all its successor

  const TransitionSet empty_set_;  // allocated address in order to be able to return an empty set

  mutable std::atomic_ulong hits_;
  mutable std::atomic_ulong miss_;
};
}  // namespace l3_footstep_planning

#endif
