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

#ifndef L3_FOOTSTEP_PLANNING_STATE_DATABASE_H__
#define L3_FOOTSTEP_PLANNING_STATE_DATABASE_H__

#include <unordered_map>

#include <l3_footstep_planning_libs/typedefs.h>
#include <l3_footstep_planning_libs/hash.h>

#include <l3_footstep_planning_libs/modeling/state.h>
#include <l3_footstep_planning_libs/modeling/state_id.h>

namespace l3_footstep_planning
{
using namespace l3;

/**
 * @brief The StateDataBase class
 */
class StateDataBase
{
private:
  typedef std::unordered_map<StateID, StateHashed::ConstPtr, StateHasher> StateMap;

public:
  // typedefs
  typedef l3::SharedPtr<StateDataBase> Ptr;
  typedef l3::SharedPtr<const StateDataBase> ConstPtr;

  /**
   * @brief Database for storing discrete states.
   * @param resolution Resolution [m/per cell] of discrete space
   */
  StateDataBase(const DiscreteResolution& resolution);

  /**
   * @brief Returns state based on given input (continous space) footholds when available.
   * @param FootholdsType type of input footholds that can be from any type, which can be also converted by StateID as first argument
   * @param footholds input list of footholds to look up
   * @param FloatingBasesType type of input floating_bases
   * @param floating_bases input list of floating bases to look up
   * @return Pointer to the state based on given input (continous space) footholds when available; otherwise nullptr
   */
  template<typename FootholdsType, typename FloatingBasesType>
  inline StateHashed::ConstPtr operator()(const FootholdsType& footholds, const FloatingBasesType floating_bases) const { return get(footholds, floating_bases); }

  /**
   * @brief Returns state based on given input (continous space) footholds when available.
   * Note that StateKey can be used as input argument but it is less efficient than using a list of foodholds.
   * @param KeyType type of input, which can be converted to a StateID
   * @param key input key to be converted into a StateID
   * @return Pointer to the state based on given input (continous space) footholds when available; otherwise nullptr
   */
  template<typename KeyType>
  inline StateHashed::ConstPtr operator()(const KeyType& key) const { return get(key); }

  /**
   * @brief Returns state based on given input (continous space) footholds when available.
   * @param UID unique ID of state
   * @return Pointer to the state based on given input (continous space) footholds when available; otherwise nullptr
   */
  inline StateHashed::ConstPtr operator()(const UID& uid) const { return get(uid); }

  /**
   * @brief Returns state based on given input (continous space) footholds when available.
   * Note that StateKey can be used as input argument but it is less efficient than using a list of foodholds.
   * @param FootholdsType type of input footholds that can be from any type, which can be also converted by StateID as first argument
   * @param footholds input list of footholds to look up
   * @return Pointer to the state based on given input (continous space) footholds when available; otherwise nullptr
   */
  template<typename KeyType>
  inline StateHashed::ConstPtr operator[](const KeyType& key) const { return get(key); }
  inline StateHashed::ConstPtr operator[](const UID& uid) const { return get(uid); }

  /**
   * @brief Number of states stored in the database.
   * @return Number of stored states
   */
  size_t size() const;

  /**
   * @brief Checks if database is empty
   * @return True, if database is empty
   */
  bool empty() const;

  /**
   * @brief Clears database
   */
  void clear();

  /**
   * @brief Inserts given state into database.
   * @param state State to be inserted
   * @return Pointer to state as stored in db
   */
  StateHashed::ConstPtr insert(const State& state);
  StateHashed::ConstPtr insert(State&& state);
  StateHashed::ConstPtr insert(State::ConstPtr state);

  /**
   * @brief Checks if state with specific uid is in database.
   * @param uid uid of state to look up
   * @return true, if corresponding foothold is in database
   */
  bool has(const UID& uid) const;

  /**
   * @brief Checks if state based on given input (continous space) footholds and floating bases is already in database.
   * Note that StateKey can be used as input argument but it is less efficient than using a list of foodholds and bases.
   * @param FootholdsType type of input footholds that can be from any type, which can be also converted by StateID as first argument
   * @param FloatingBasesType type of input bases that can be from any type, which can be also converted by StateID as first argument
   * @param footholds input list of footholds to look up
   * @param floating_bases input list of bases to look up
   * @return true, if corresponding state is in database
   */
  template<typename KeyType>
  inline bool has(const KeyType& key) const { return has(StateID(key, resolution_)); }

  template<typename FootholdsType, typename FloatingBaseType>
  inline bool has(const FootholdsType& footholds, const FloatingBaseType& floating_bases) const { return has(StateID(footholds, floating_bases, resolution_)); }
  inline bool has(const FootholdHashedConstPtrArray& footholds, const FloatingBaseHashedConstPtrArray& floating_bases) const { return has(StateID(footholds, floating_bases)); }
  inline bool has(const State& state) const { return has(StateID(state)); }

  /**
   * @brief Returns the state associated with the input UID when available
   * @param uid uid of state to look up
   * @return Pointer to the state associated with the input UID when available; otherwise nullptr
   */
  StateHashed::ConstPtr get(const UID& uid) const;

  /**
   * @brief Returns state based on given input (continous space) footholds and floating bases when available.
   * Note that StateKey can be used as input argument but it is less efficient than using a list of foodholds and bases.
   * @param FootholdsType type of input footholds that can be from any type, which can be also converted by StateID as first argument
   * @param FloatingBasesType type of input bases that can be from any type, which can be also converted by StateID as first argument
   * @param footholds input list of footholds to look up
   * @param floating_bases input list of bases to look up
   * @return Pointer to the state based on given input (continous space) footholds and bases when available; otherwise nullptr
   */
  template<typename FootholdsType, typename FloatingBasesType>
  inline StateHashed::ConstPtr get(const FootholdsType& footholds, const FloatingBasesType& floating_bases) const { return get(StateID(footholds, floating_bases, resolution_)); }
  inline StateHashed::ConstPtr get(const FootholdHashedConstPtrArray& footholds, const FloatingBaseHashedConstPtrArray& floating_bases) const { return get(StateID(footholds, floating_bases)); }
  inline StateHashed::ConstPtr get(const State& state) const { return get(StateID(state)); }

  template<typename KeyType>
  inline StateHashed::ConstPtr get(const KeyType& key) const { return get(StateID(key, resolution_)); }

  /**
   * @brief Removes state based on given input (continous space) footholds and floating bases.
   * @param FootholdsType type of input footholds that can be from any type, which can be also converted by StateID as first argument
   * @param FloatingBasesType type of input bases that can be from any type, which can be also converted by StateID as first argument
   * @param footholds input list of footholds to look up
   * @param floating_bases input list of bases to look up
   */
  template<typename FootholdsType, typename FloatingBasesType>
  inline void remove(const FootholdsType& footholds, const FloatingBasesType& floating_bases) { remove(StateID(footholds, floating_bases, resolution_)); }
  inline void remove(const FootholdHashedConstPtrArray& footholds, const FloatingBaseHashedConstPtrArray& floating_bases) { remove(StateID(footholds, floating_bases)); }

  /**
   * @brief Returns statistical performance information. The database logs the number of successful
   * caching operations.
   * @return Number of successful hits
   */
  inline unsigned long hits() const { return hits_; }

  /**
   * @brief Returns statistical performance information. The database logs the number of successful
   * caching operations.
   * @return Number of misses
   */
  inline unsigned long miss() const { return miss_; }

  /**
   * @brief Returns resolution used for discretization by the database
   * @return Resolution of discretization grid
   */
  inline const DiscreteResolution& getResolution() const { return resolution_; }

protected:
  bool has(const StateID& id) const;

  StateHashed::ConstPtr get(const StateID& id) const;

  void remove(const StateID& id);

  mutable Mutex mutex_;

  DiscreteResolution resolution_;

  StateMap map_;
  std::vector<StateHashed::ConstPtr> uid_to_state_;

  mutable std::atomic_ulong hits_;
  mutable std::atomic_ulong miss_;
};
}  // namespace l3_footstep_planning

#endif
