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

#ifndef L3_FOOTSTEP_PLANNING_STATE_ID_H__
#define L3_FOOTSTEP_PLANNING_STATE_ID_H__

#include <l3_footstep_planning_libs/typedefs.h>
#include <l3_footstep_planning_libs/hash.h>

#include <l3_footstep_planning_libs/modeling/state.h>
#include <l3_footstep_planning_libs/modeling/foothold_id.h>
#include <l3_footstep_planning_libs/modeling/floating_base_id.h>

namespace l3_footstep_planning
{
using namespace l3;

/**
 * @brief The StateKey struct consists only of the main components uniquely identifying a
 * state within the (discretized) database.
 */
struct StateKey
{
  inline StateKey()
  {}

  inline StateKey(const FootholdArray& footholds, const FloatingBaseArray& floating_bases)
    : footholds(footholds)
    , floating_bases(floating_bases)
  {}

  inline StateKey(const FootholdConstPtrArray& footholds, const FloatingBaseConstPtrArray& floating_bases)
  {
    for (Foothold::ConstPtr f : footholds)
      this->footholds.push_back(*f);

    for (FloatingBase::ConstPtr fb : floating_bases)
      this->floating_bases.push_back(*fb);
  }

  FootholdArray footholds;
  FloatingBaseArray floating_bases;
};

/**
 * @brief The StateId represents a State in discrete space. It holds the concatenated
 * FootholdIds of each state's foothold and FloatingBaseIds of each state's floating base
 */
class StateID : public Eigen::VectorXi
{
public:
  StateID()
    : Eigen::VectorXi()
    , num_footholds_(0)
    , num_floating_bases_(0)
  {
    setZero();
  }

  /**
   * @brief Plain initialization for StateID.
   * @param vector_list_feet List of FootholdIDs given as Eigen::Vector
   * @param vector_list_bases List of FloatingBaseIDs given as Eigen::Vector
   */
  StateID(const std::vector<EigenFootholdID>& vector_list_feet, const std::vector<EigenFloatingBaseID>& vector_list_bases)
    : Eigen::VectorXi(stateSize(vector_list_feet.size(), vector_list_bases.size()))
    , num_footholds_(vector_list_feet.size())
    , num_floating_bases_(vector_list_bases.size())
  {
    for (size_t i = 0; i < vector_list_feet.size(); i++)
      this->segment<FOOTHOLD_ID_SIZE>(footholdStartIndex(i)) = vector_list_feet[i];

    for (size_t i = 0; i < vector_list_bases.size(); i++)
      this->segment<FLOATING_BASE_ID_SIZE>(floatingBaseStartIndex(i)) = vector_list_bases[i];

    hash_ = hash_value(*this);
  }

  /**
   * @brief Creates StateID from existing state key.
   * @param state State for which to compute the StateID
   * @param resolution Grid cell resolution used for discretization
   */
  inline StateID(const StateKey& key, const DiscreteResolution& resolution)
    : Eigen::VectorXi(stateSize(key.footholds.size(), key.floating_bases.size()))
    , num_footholds_(key.footholds.size())
    , num_floating_bases_(key.floating_bases.size())
  {
    for (size_t i = 0; i < key.footholds.size(); i++)
      FootholdID::getFoodholdId(*this, footholdStartIndex(i),
                                key.footholds[i].idx,
                                key.footholds[i].x(), key.footholds[i].y(), key.footholds[i].z(),
                                key.footholds[i].yaw(),
                                resolution);

    for(size_t i = 0; i < key.floating_bases.size(); i++)
      FloatingBaseID::getFloatingBaseId(*this, floatingBaseStartIndex(i),
                                        key.floating_bases[i].idx,
                                        key.floating_bases[i].x(), key.floating_bases[i].y(), key.floating_bases[i].z(),
                                        key.floating_bases[i].roll(), key.floating_bases[i].pitch(), key.floating_bases[i].yaw(),
                                        resolution);
    boost::hash<StateID> hasher;
    hash_ = hasher(*this);
  }

  /**
   * @brief Creates StateID from existing state. This variant is the most
   * efficient option for computing the state's hash as it take use of the cached
   * hash values from the footholds and floating bases stored in the state.
   * @param state State to compute the StateID from
   */
  inline StateID(const State& state)
    : StateID(state.getFootholdsHashed(), state.getFloatingBasesHashed())
  {}

  /**
   * @brief Creates StateID from existing state. This variant is the most
   * efficient option for computing the state's hash as it take use of the cached
   * hash values from the footholds and floating bases stored in the state.
   * @param state State to compute the StateID from
   */
  inline StateID(State::ConstPtr state)
    : StateID(*state)
  {}

  /**
   * @brief Creates StateID from given input hashed footholds and floating bases.
   * @param footholds Array of footholds
   * @param floating_bases Array of floating bases
   * @param resolution Grid cell resolution used for discretization
   */
  StateID(const FootholdArray& footholds, const FloatingBaseArray& floating_bases, const DiscreteResolution& resolution)
    : Eigen::VectorXi(stateSize(footholds.size(), floating_bases.size()))
    , num_footholds_(footholds.size())
    , num_floating_bases_(floating_bases.size())
  {
    for (size_t i = 0; i < footholds.size(); i++)
      FootholdID::getFoodholdId(*this, footholdStartIndex(i),
                                footholds[i].idx,
                                footholds[i].x(), footholds[i].y(), footholds[i].z(),
                                footholds[i].yaw(),
                                resolution);

    for(size_t i = 0; i < floating_bases.size(); i++)
      FloatingBaseID::getFloatingBaseId(*this, floatingBaseStartIndex(i),
                                        floating_bases[i].idx,
                                        floating_bases[i].x(), floating_bases[i].y(), floating_bases[i].z(),
                                        floating_bases[i].roll(), floating_bases[i].pitch(), floating_bases[i].yaw(),
                                        resolution);

    boost::hash<StateID> hasher;
    hash_ = hasher(*this);
  }

  /**
   * @brief Creates StateID from given input hashed footholds and floating bases.
   * @param footholds Array of const pointers to footholds
   * @param floating_bases Array of const pointers to floating bases
   * @param resolution Grid cell resolution used for discretization
   */
  StateID(const FootholdConstPtrArray& footholds, FloatingBaseConstPtrArray floating_bases, const DiscreteResolution& resolution)
    : Eigen::VectorXi(stateSize(footholds.size(), floating_bases.size()))
    , num_footholds_(footholds.size())
    , num_floating_bases_(floating_bases.size())
  {
    for (size_t i = 0; i < footholds.size(); i++)
      FootholdID::getFoodholdId(*this, footholdStartIndex(i),
                                footholds[i]->idx,
                                footholds[i]->x(), footholds[i]->y(), footholds[i]->z(),
                                footholds[i]->yaw(),
                                resolution);

    for(size_t i = 0; i < floating_bases.size(); i++)
      FloatingBaseID::getFloatingBaseId(*this, floatingBaseStartIndex(i),
                                        floating_bases[i]->idx,
                                        floating_bases[i]->x(), floating_bases[i]->y(), floating_bases[i]->z(),
                                        floating_bases[i]->roll(), floating_bases[i]->pitch(), floating_bases[i]->yaw(),
                                        resolution);

    boost::hash<StateID> hasher;
    hash_ = hasher(*this);
  }

  /**
   * @brief Creates StateID from given input hashed footholds and floating bases. This variant is the most
   * efficient option for computing the state's hash as it take use of the cached
   * values from the footholds and floating bases.
   * @param footholds Array of const pointers to hashed footholds
   * @param floating_bases Array of const pointers to hashed floating bases
   */
  StateID(const FootholdHashedConstPtrArray& footholds, FloatingBaseHashedConstPtrArray floating_bases)
    : Eigen::VectorXi(stateSize(footholds.size(), floating_bases.size()))
    , num_footholds_(footholds.size())
    , num_floating_bases_(floating_bases.size())
  {
    hash_ = 0;

    for (size_t i = 0; i < footholds.size(); i++)
    {
      this->segment<FOOTHOLD_ID_SIZE>(footholdStartIndex(i)) = footholds[i]->getID();
      boost::hash_combine(hash_, footholds[i]->getHashValue());
    }

    for (size_t i = 0; i < floating_bases.size(); i++)
    {
      this->segment<FLOATING_BASE_ID_SIZE>(floatingBaseStartIndex(i)) = floating_bases[i]->getID();
      boost::hash_combine(hash_, floating_bases[i]->getHashValue());
    }
  }

  inline bool operator==(const StateID& other) const { return hash_ == other.hash_ && static_cast<const Eigen::VectorXi&>(*this) == static_cast<const Eigen::VectorXi&>(other); }

  /** Specialization for boost::hash */
  friend Hash hash_value(const StateID& id)
  {    
    assert(id.size() == stateSize(id.num_footholds_, id.num_floating_bases_));

    Hash seed = 0;

    for (size_t i = 0; i < id.num_footholds_; i++)
      boost::hash_combine(seed, FootholdID(id.segment<FOOTHOLD_ID_SIZE>(footholdStartIndex(i))).getHashValue());

    for (size_t i = 0; i < id.num_floating_bases_; i++)
      boost::hash_combine(seed, FloatingBaseID(id.segment<FLOATING_BASE_ID_SIZE>(floatingBaseStartIndex(i, id.num_footholds_))).getHashValue());

    return seed;
  }

  inline const Hash& getHashValue() const { return hash_; }

private:
  static size_t stateSize(size_t num_footholds, size_t num_floating_bases = 0) { return num_footholds * FOOTHOLD_ID_SIZE + num_floating_bases * FLOATING_BASE_ID_SIZE; }

  static Eigen::Index footholdStartIndex(size_t num_foothold) { return static_cast<long>(num_foothold) * FOOTHOLD_ID_SIZE; }

  inline Eigen::Index floatingBaseStartIndex(size_t num_floating_base) const { return floatingBaseStartIndex(num_floating_base, num_footholds_); }
  static Eigen::Index floatingBaseStartIndex(size_t num_floating_base, size_t num_footholds) { return footholdStartIndex(num_footholds) + static_cast<long>(num_floating_base) * FLOATING_BASE_ID_SIZE; }

  Hash hash_;

  size_t num_footholds_;
  size_t num_floating_bases_;
};

typedef Hashed<State, StateID> StateHashed;
typedef Hasher<StateID> StateHasher;  // Hashing operator

L3_STATIC_ASSERT_MOVEABLE(StateID)
L3_STATIC_ASSERT_MOVEABLE(StateHashed)
}  // namespace l3_footstep_planning

#endif
