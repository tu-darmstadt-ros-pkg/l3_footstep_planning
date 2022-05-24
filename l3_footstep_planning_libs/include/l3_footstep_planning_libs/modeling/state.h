//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, TU Darmstadt
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

#ifndef FOOTSTEP_PLANNER_STATE_H__
#define FOOTSTEP_PLANNER_STATE_H__

#include <mutex>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include <l3_libs/types/types.h>

#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

#include <l3_footstep_planning_libs/math.h>
#include <l3_footstep_planning_libs/hash.h>

#include <l3_footstep_planning_libs/modeling/foothold_database.h>
#include <l3_footstep_planning_libs/modeling/floating_base_id.h>

namespace l3_footstep_planning
{
using namespace l3;

/**
 * @brief A class representing the robot's state (current footholds and floating bases) in the (continuous) world view.
 */
class State
{
public:
  // typedefs
  typedef std::pair<const FootIndex, FootholdHashed::ConstPtr> FootholdPair;
  typedef std::map<FootIndex, FootholdHashed::ConstPtr> FootholdMap;

  typedef std::pair<const BaseIndex, FloatingBaseHashed::ConstPtr> FloatingBasePair;
  typedef std::map<BaseIndex, FloatingBaseHashed::ConstPtr> FloatingBaseMap;

  typedef l3::SharedPtr<State> Ptr;
  typedef l3::SharedPtr<const State> ConstPtr;

  State();

  State(const FootholdHashedConstPtrArray& footholds, const FloatingBaseHashedConstPtrArray& floating_bases = FloatingBaseHashedConstPtrArray());

  State(const State& other)
    : State(other, SharedLock(other.mutex_))
  {}

  State(State&& other)
    : State(std::move(other), UniqueLock(other.mutex_))
  {}

protected:
  /**
   * @brief Thread-safe copy constructor
   */
  State(const State& other, SharedLock other_lock);

  /**
   * @brief Thread-safe move constructor
   */
  State(State&& other, UniqueLock other_lock);

public:
  /**
   * @brief Thread-safe swap operator
   */
  State& swap(State& other);

  /**
   * @brief Thread-safe copy operator
   */
  State& operator=(const State& other);

  /**
   * @brief Thread-safe move operator
   */
  State& operator=(State&& other);

  /**
   * @brief Compare two states on equality based on footholds
   */
  bool operator==(const State& other) const;

  /**
   * @brief Inequality operator for two states (negates the equality operator).
   */
  bool operator!=(const State& other) const;

  /**
   * @brief Clears all stored data
   */
  void clear();

  /**
   * @brief Clears all stored footholds
   */
  void clearFootholds();

  /**
   * @brief Checks if footholds are available
   * @return True, if footholds are available
   */
  bool hasFootholds() const;

  void updateFoothold(FootholdHashed::ConstPtr foothold);

  void updateFootholds(FootholdHashedConstPtrArray footholds);

  const State::FootholdMap& getFooholdMap() const { return foothold_map_; }

  FootholdHashed::ConstPtr getFoothold(const FootIndex& foot_idx) const;

  const FootholdHashedConstPtrArray& getFootholdsHashed() const { return footholds_hashed_; }

  const FootholdConstPtrArray& getFootholds() const { return footholds_; }

  /**
   * @brief Returns all changed footholds compared to another state. A foothold is declared
   * as changed when its foot idx occurs in both states and their UID differ.
   * @param other State from which all changed footholds should be collected
   * @return List of all changed footholds, taken from other state
   */
  FootholdHashedConstPtrArray getChangedFootholds(const State& other) const;
  FootIndexSet getChangedFootholdIdx(const State& other) const;

  /**
   * @brief Returns all unchanged footholds compared to another state. A foothold is declared
   * as unchanged when its foot idx occurs in both states and their UID equal.
   * @param other State from which all unchanged footholds should be collected
   * @return List of all changed footholds, taken from other state
   */
  FootholdHashedConstPtrArray getUnchangedFootholds(const State& other) const;
  FootIndexSet getUnchangedFootholdIdx(const State& other) const;

  /**
   * @brief Clears floating base data
   */
  void clearFloatingBases();

  /**
   * @brief Checks if floating bases are available
   * @return True, if floating bases are available
   */
  bool hasFloatingBases() const;

  void updateFloatingBase(FloatingBaseHashed::ConstPtr floating_base);

  void updateFloatingBases(FloatingBaseHashedConstPtrArray floating_bases);

  const State::FloatingBaseMap& getFloatingBaseMap() const { return floating_base_map_; }

  FloatingBaseHashed::ConstPtr getFloatingBase(const BaseIndex& base_idx) const;

  const FloatingBaseHashedConstPtrArray& getFloatingBasesHashed() const { return floating_bases_hashed_; }

  const FloatingBaseConstPtrArray& getFloatingBases() const { return floating_bases_; }

  /**
   * @brief Returns all changed floating bases compared to another state. A floating base is declared
   * as changed when its base idx occurs in both states and their UID differ.
   * @param other State from which all changed floating bases should be collected
   * @return List of all changed floating bases, taken from other state
   */
  FloatingBaseHashedConstPtrArray getChangedFloatingBases(const State& other) const;
  BaseIndexSet getChangedFloatingBaseIdx(const State& other) const;

  /**
   * @brief Returns all unchanged floating bases compared to another state. A floating base is declared
   * as unchanged when its base idx occurs in both states and their UID equal.
   * @param other State from which all unchanged floating bases should be collected
   * @return List of all changed floating bases, taken from other state
   */
  FloatingBaseHashedConstPtrArray getUnchangedFloatingBases(const State& other) const;
  BaseIndexSet getUnchangedFloatingBaseIdx(const State& other) const;

  /**
   * @brief Returns feet center associated with this state
   * @return current feet center
   */
  const Pose& getFeetCenter() const;

  /**
   * @brief Defines if this state is a start state
   */
  void setIsStart(bool is_start) { is_start_ = is_start; }
  inline bool isStart() const { return is_start_; }

  /**
   * @brief Defines if this state is a goal state
   */
  void setIsGoal(bool is_goal) { is_goal_ = is_goal; }
  inline bool isGoal() const { return is_goal_; }

  /**
   * @brief States are getting cached even they are invalid (e.g. kinematically not reachable or colliding).
   * This flag indentifies such states.
   */
  void setValid(bool valid) { valid_ = valid; }
  inline bool isValid() const { return valid_; }

protected:
  void rebuildFeetLists();
  void rebuildFloatingBasesLists();

  mutable Mutex mutex_;

  // state defined by footholds (stored in different containers for efficient access)
  State::FootholdMap foothold_map_;
  FootholdHashedConstPtrArray footholds_hashed_;
  FootholdConstPtrArray footholds_;

  // state further defined by floating bases (stored in different containers for efficient access)
  State::FloatingBaseMap floating_base_map_;
  FloatingBaseHashedConstPtrArray floating_bases_hashed_;
  FloatingBaseConstPtrArray floating_bases_;

  // indicates if start or goal state
  bool is_start_;
  bool is_goal_;

  mutable bool feet_center_dirty_ = true;  // dirty flag for robot center computation
  mutable Pose feet_center_;

  bool valid_;  // inidicates if this state has passed validity checks (states are getting cached, although they may not be valid)

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

L3_STATIC_ASSERT_MOVEABLE(State)
}  // namespace l3_footstep_planning
#endif
