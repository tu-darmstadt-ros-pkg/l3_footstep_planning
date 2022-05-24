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

#ifndef L3_FOOTSTEP_PLANNING_STATE_SPACE_MANAGER_H__
#define L3_FOOTSTEP_PLANNING_STATE_SPACE_MANAGER_H__

#include <vigir_generic_params/parameter_set.h>

#include <l3_libs/types/typedefs.h>
#include <l3_libs/singleton.h>

#include <l3_footstep_planning_libs/modeling/foothold_database.h>
#include <l3_footstep_planning_libs/modeling/floating_base_database.h>
#include <l3_footstep_planning_libs/modeling/state_database.h>
#include <l3_footstep_planning_libs/modeling/transition_database.h>
#include <l3_footstep_planning_libs/modeling/planning_state.h>

namespace l3_footstep_planning
{
class StateSpaceManager : public Singleton<StateSpaceManager>
{
private:
  typedef std::unordered_map<PlanningStateID, PlanningStateHashed::ConstPtr, PlanningStateHasher> PlanningStateMap;

public:
  StateSpaceManager();

  /**
   * @brief Initialize interface manager
   * @param plugin_set_name Name of plugin set to load
   */
  static bool initialize(const vigir_generic_params::ParameterSet& params);

  static void setSbplEnvironment(std::vector<int*>& state_ID2_index_mapping);

  inline static const DiscreteResolution& getResolution() { return instance().res_; }

  static void clear();

  // basic sub-database access
  inline FootholdHashed::ConstPtr operator()(const FootholdKey& key) const { return getFoothold(key); }
  inline StateHashed::ConstPtr operator()(const UID& uid) const { return getState(uid); }
  inline StateHashed::ConstPtr operator()(const StateKey& key) const { return getState(key); }
  inline StateHashed::ConstPtr operator()(const FootholdConstPtrArray& footholds, const FloatingBaseConstPtrArray& floating_bases) const { return getState(footholds, floating_bases); }
  inline StateHashed::ConstPtr operator()(const FootholdHashedConstPtrArray& footholds, const FloatingBaseHashedConstPtrArray& floating_bases) const { return getState(footholds, floating_bases); }
  inline TransitionHashed::ConstPtr operator()(const StateID& pred, const StateID& succ) const { return getTransition(pred, succ); }
  inline TransitionHashed::ConstPtr operator()(const TransitionID& id) const { return getTransition(id); }

  inline FootholdHashed::ConstPtr operator[](const FootholdKey& key) const { return getFoothold(key); }
  inline StateHashed::ConstPtr operator[](const UID& uid) const { return getState(uid); }
  inline StateHashed::ConstPtr operator[](const StateKey& key) const { return getState(key); }
  inline TransitionHashed::ConstPtr operator[](const TransitionID& id) const { return getTransition(id); }

  /// Foothold DB

  /**
   * @brief Adds single foothold to database.
   * @param foothold Foothold to be added
   * @return Discretized version of input foothold as stored in database.
   */
  inline static FootholdHashed::ConstPtr addFoothold(Foothold::ConstPtr foothold) { return addFoothold(*foothold); }
  static FootholdHashed::ConstPtr addFoothold(const Foothold& foothold);

  /**
   * @brief Adds multiple foothold to database.
   * @param footholds Footholds to be added
   * @return List of discretized version of input footholds as stored in database.
   */
  static FootholdHashedConstPtrArray addFootholds(const FootholdConstPtrArray& footholds);
  static FootholdHashedConstPtrArray addFootholds(const FootholdArray& footholds);

  /**
   * @brief Checks if discretized version of given input (continous space) foothold is already in database.
   * @param KeyType type of input key that can be from any type, which can be also converted by FootholdID
   * @param key input key to look up
   * @return true, if corresponding foothold is in database
   */
  template<typename KeyType>
  static bool hasFoothold(const KeyType& key)
  {
    SharedLock lock(instance().mutex_);
    return instance().foothold_db_->has(key);
  }

  /**
   * @brief Returns discretized version of given input (continous space) foothold when available.
   * @param KeyType type of input key that can be from any type, which can be also converted by FootholdID
   * @param key input key to look up
   * @return Pointer to discretized version of given input foothold when available; otherwise nullptr
   */
  template<typename KeyType>
  static FootholdHashed::ConstPtr getFoothold(const KeyType& key)
  {
    SharedLock lock(instance().mutex_);
    return instance().foothold_db_->get(key);
  }

  /**
   * @brief Number of footholds stored in the database.
   * @return Number of stored footholds
   */
  inline static size_t getNumFootholds() { return instance().foothold_db_->size(); }

  inline static unsigned long footholdHits() { return instance().foothold_db_->hits(); }
  inline static unsigned long footholdMiss() { return instance().foothold_db_->miss(); }

  /// Floating Base DB

  /**
   * @brief Adds single floating base to database.
   * @param floating_base Floating base to be added
   * @return Discretized version of input floating base as stored in database.
   */
  inline static FloatingBaseHashed::ConstPtr addFloatingBase(FloatingBase::ConstPtr floating_base) { return addFloatingBase(*floating_base); }
  static FloatingBaseHashed::ConstPtr addFloatingBase(const FloatingBase& floating_base);

  /**
   * @brief Adds multiple floating bases to database.
   * @param floating_bases Floating bases to be added
   * @return List of discretized version of input floating bases as stored in database.
   */
  static FloatingBaseHashedConstPtrArray addFloatingBases(const FloatingBaseConstPtrArray& floating_bases);
  static FloatingBaseHashedConstPtrArray addFloatingBases(const FloatingBaseArray& floating_bases);

  /**
   * @brief Checks if discretized version of given input (continous space) floating base is already in database.
   * @param KeyType type of input key that can be from any type, which can be also converted by FloatingBaseID
   * @param key input key to look up
   * @return true, if corresponding floating base is in database
   */
  template<typename KeyType>
  static bool hasFloatingBase(const KeyType& key)
  {
    SharedLock lock(instance().mutex_);
    return instance().floating_base_db_->has(key);
  }

  /**
   * @brief Returns discretized version of given input (continous space) floating base when available.
   * @param KeyType type of input key that can be from any type, which can be also converted by FloatingBaseID
   * @param key input key to look up
   * @return Pointer to discretized version of given input floating base when available; otherwise nullptr
   */
  template<typename KeyType>
  static FloatingBaseHashed::ConstPtr getFloatingBase(const KeyType& key)
  {
    SharedLock lock(instance().mutex_);
    return instance().floating_base_db_->get(key);
  }

  /**
   * @brief Number of floating bases stored in the database.
   * @return Number of stored floating bases
   */
  inline static size_t getNumFloatingBases() { return instance().floating_base_db_->size(); }

  inline static unsigned long floatingBaseHits() { return instance().floating_base_db_->hits(); }
  inline static unsigned long floatingBaseMiss() { return instance().floating_base_db_->miss(); }

  /// State DB

  /**
   * @brief Adds new state to the database created from input footholds. Footholds are
   * automatically added to the database as well.
   * @param FootholdsType type of list of footholds
   * @param footholds list of footholds that are forming the new state
   * @param FloatingBasesType type of list of floating bases
   * @param floating_bases list of floating bases that are forming the new state
   * @return Added (or equally found) state based on given input
   */
  template<typename FootholdsType, typename FloatingBasesType>
  static StateHashed::ConstPtr createState(const FootholdsType& footholds, const FloatingBasesType& floating_bases)
  {
    SharedLock lock(instance().mutex_);
    return addState(State(addFootholds(footholds), addFloatingBases(floating_bases)));
  }

  /**
   * @brief Adds new state to the database created from input footholds and floating bases.
   * @param footholds list of footholds that are forming the new state
   * @param floating_bases list of bases that are forming the new state
   * @return Added (or equally found) state based on given input
   */
  static StateHashed::ConstPtr createState(const FootholdHashedConstPtrArray& footholds, const FloatingBaseHashedConstPtrArray& floating_bases = FloatingBaseHashedConstPtrArray())
  {
    SharedLock lock(instance().mutex_);
    return addState(State(footholds, floating_bases));
  }

  /**
   * @brief Adds new state to the database.
   * @param state State to be added
   * @return Added (or equally found) state based on given input
   */
  inline static StateHashed::ConstPtr addState(State::ConstPtr state) { return addState(state); }
  static StateHashed::ConstPtr addState(const State& state);
  static StateHashed::ConstPtr addState(State&& state);

  /**
   * @brief Checks if state based on given input key is already in database.
   * Note that StateKey can be used as input argument but it is less efficient than using a list of foodholds.
   * @param KeyType type of input footholds that can be from any type, which can be also converted by StateID as first argument
   * @param key input key to look up (e.g. StateKey or FootholdArray)
   * @return true, if corresponding state is in database
   */
  template<typename KeyType>
  static bool hasState(const KeyType& key)
  {
    SharedLock lock(instance().mutex_);
    return instance().state_db_->has(key);
  }

  /**
   * @brief Checks if state based on given input (continous space) footholds is already in database.
   * @param FootholdsType type of input footholds that can be from any type, which can be also converted by StateID as first argument
   * @param footholds input list of footholds to look up
   * @param FloatingBaseType type of input floating base that can be from any type, which can be also converted by StateID as second argument
   * @param floating_base input floating base to look up
   * @return true, if corresponding state is in database
   */
  template<typename FootholdsType, typename FloatingBaseType>
  static bool hasState(const FootholdsType& footholds, const FloatingBaseType& floating_base)
  {
    SharedLock lock(instance().mutex_);
    return instance().state_db_->has(footholds, floating_base);
  }

  /**
   * @brief Returns state based on given input key when available.
   * Note that StateKey can be used as input argument but it is less efficient than using a list of foodholds.
   * @param KeyType type of input footholds that can be from any type, which can be also converted by StateID as first argument
   * @param key input key to look up (e.g. UID, StateKey or FootholdArray)
   * @return Pointer to the state based on given input (continous space) footholds when available; otherwise nullptr
   */
  template<typename KeyType>
  static StateHashed::ConstPtr getState(const KeyType& key)
  {
    SharedLock lock(instance().mutex_);
    return instance().state_db_->get(key);
  }

  /**
   * @brief Returns state based on given input foothold and base keys when available.
   * Note that StateKey can be used as input argument but it is less efficient than using a list of foodholds and bases.
   * @param FootholdsType type of input footholds that can be from any type, which can be also converted by FootholdID as first argument
   * @param FloatingBasesType type of input floating bases that can be from any type, which can be also converted by FloatingBaseID as first argument
   * @param key input key to look up (e.g. UID, StateKey or FootholdArray)
   * @return Pointer to the state based on given input (continous space) footholds and bases when available; otherwise nullptr
   */
  template<typename FootholdsType, typename FloatingBasesType>
  static StateHashed::ConstPtr getState(const FootholdsType& footholds, const FloatingBasesType& floating_bases)
  {
    SharedLock lock(instance().mutex_);
    return instance().state_db_->get(footholds, floating_bases);
  }

  /**
   * @brief Number of states stored in the database.
   * @return Number of stored states
   */
  inline static size_t getNumStates() { return instance().state_db_->size(); }

  inline static unsigned long stateHits() { return instance().state_db_->hits(); }
  inline static unsigned long stateMiss() { return instance().state_db_->miss(); }

  /// Transition DB

  static TransitionHashed::ConstPtr addTransition(const Transition& transition);
  static TransitionHashed::ConstPtr addTransition(Transition&& transition);

  static TransitionHashed::ConstPtr getTransition(const StateID& pred, const StateID& succ);
  static TransitionHashed::ConstPtr getTransition(const TransitionID& id);
  static TransitionSet getPredecessors(const StateID& id);
  static TransitionSet getSuccessors(const StateID& id);

  /**
   * @brief Number of transitions stored in the database.
   * @return Number of transitions states
   */
  inline static size_t getNumTransitions() { return instance().transition_db_->size(); }

  inline static unsigned long transitionHits() { return instance().transition_db_->hits(); }
  inline static unsigned long transitionMiss() { return instance().transition_db_->miss(); }

  /// Planning State DB

  // clang-format off
  // planning state management
  inline static PlanningStateHashed::ConstPtr createPlanningState(StateHashed::ConstPtr state,
                                                                  StateHashed::ConstPtr pred = StateHashed::ConstPtr(),
                                                                  StateHashed::ConstPtr succ = StateHashed::ConstPtr(),
                                                                  Step::ConstPtr step = Step::ConstPtr())
  {
    return createPlanningState(PlanningState(state, pred, succ, step));
  }
  inline static PlanningStateHashed::ConstPtr createPlanningState(const State& state, const State& pred, const State& succ, Step::ConstPtr step = Step::ConstPtr())
  {
    return createPlanningState(PlanningState(addState(state), addState(pred), addState(succ), step));
  }
  inline static PlanningStateHashed::ConstPtr createPlanningState(const State& state) { return createPlanningState(PlanningState(addState(state))); }
  inline static PlanningStateHashed::ConstPtr createPlanningState(State&& state) { return createPlanningState(PlanningState(addState(std::move(state)))); }
  // clang-format on

  static PlanningStateHashed::ConstPtr createPlanningState(const PlanningState& pstate);
  static PlanningStateHashed::ConstPtr createPlanningState(PlanningState&& pstate);

  static PlanningStateHashed::ConstPtr getPlanningState(const UID& uid);
  inline static PlanningStateHashed::ConstPtr getPlanningState(const PlanningState& pstate) { return instance().getPlanningState(PlanningStateID(pstate)); }
  inline static PlanningStateHashed::ConstPtr getPlanningState(StateHashed::ConstPtr state, StateHashed::ConstPtr pred, StateHashed::ConstPtr succ, const FootIndexSet& moved_feet_idx, const BaseIndexSet& moved_bases_idx)
  {
    return instance().getPlanningState(PlanningStateID(state, pred, succ, moved_feet_idx, moved_bases_idx));
  }  

  /**
   * @brief Number of planning states stored in the database.
   * @return Number of stored planning states
   */
  inline static size_t getNumPlanningStates() { return instance().map_.size(); }

  inline static unsigned long planningStatesHits() { return instance().hits_; }
  inline static unsigned long planningStatesMiss() { return instance().miss_; }

private:
  PlanningStateHashed::ConstPtr getPlanningState(const PlanningStateID& id) const;

  void createSbplPlanningEntry(const PlanningStateHashed& pstate);

  mutable Mutex mutex_;

  // resolution used by sub-databases
  DiscreteResolution res_;

  // sub-databases
  FootholdDataBase::Ptr foothold_db_;
  FloatingBaseDataBase::Ptr floating_base_db_;
  StateDataBase::Ptr state_db_;
  TransitionDataBase::Ptr transition_db_;

  // internal planning state management
  PlanningStateMap map_;
  std::vector<PlanningStateHashed::ConstPtr> uid_to_state_;

  mutable std::atomic_ulong hits_;
  mutable std::atomic_ulong miss_;

  std::vector<int*>* state_ID2_index_mapping_;  // required by SBPL
};
}  // namespace l3_footstep_planning

#endif
