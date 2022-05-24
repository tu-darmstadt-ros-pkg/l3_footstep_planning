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

#ifndef L3_FOOTSTEP_PLANNING_TRANSITION_ID_H__
#define L3_FOOTSTEP_PLANNING_TRANSITION_ID_H__

#include <l3_footstep_planning_libs/typedefs.h>
#include <l3_footstep_planning_libs/hash.h>

#include <l3_footstep_planning_libs/modeling/transition.h>
#include <l3_footstep_planning_libs/modeling/state_id.h>

namespace l3_footstep_planning
{
using namespace l3;

/**
 * @brief The TransitionID representes an transition in order to change state from
 * predecessor to successor.
 */
class TransitionID
{
public:
  /**
   * @brief Creates TransitionID based on predecessor and successor state
   * @param pred predecessor state
   * @param succ successor state
   */
  TransitionID(const StateID& pred, const StateID& succ)
    : pred_(pred)
    , succ_(succ)
  {
    boost::hash<TransitionID> hasher;
    hash_ = hasher(*this);
  }

  inline bool operator==(const TransitionID& other) const { return hash_ == other.hash_ && pred_ == other.pred_ && succ_ == other.succ_; }

  /** Specialization for boost::hash */
  friend Hash hash_value(const TransitionID& id)
  {
    Hash seed = 0;
    boost::hash_combine(seed, id.pred_.getHashValue());
    boost::hash_combine(seed, id.succ_.getHashValue());
    return seed;
  }

  inline const StateID& getPredecessorID() const { return pred_; }
  inline const StateID& getSuccessorID() const { return succ_; }

  inline const Hash& getHashValue() const { return hash_; }

private:
  StateID pred_;
  StateID succ_;
  Hash hash_;
};

typedef Hashed<Transition, TransitionID> TransitionHashed;

// Hashing operator
struct TransitionHasher
{
  inline Hash operator()(const TransitionID& id) const { return id.getHashValue(); }

  inline Hash operator()(TransitionHashed::ConstPtr transition) const { return transition->getHashValue(); }
};

L3_STATIC_ASSERT_MOVEABLE(TransitionID)
}  // namespace l3_footstep_planning

#endif
