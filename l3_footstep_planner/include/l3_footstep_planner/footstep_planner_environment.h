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

#ifndef L3_FOOTSTEP_PLANNING_FOOTSTEP_PLANNER_ENVIRONMENT_H__
#define L3_FOOTSTEP_PLANNING_FOOTSTEP_PLANNER_ENVIRONMENT_H__

#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <sbpl/headers.h>

#include <l3_footstep_planning_libs/helper.h>

#include <l3_footstep_planning_plugins/aggregator/world_model.h>
#include <l3_footstep_planning_plugins/aggregator/step_cost_estimator.h>
#include <l3_footstep_planning_plugins/aggregator/heuristic.h>

#include <l3_footstep_planner/environment_parameters.h>
#include <l3_footstep_planner/state_space/state_space.h>
#include <l3_footstep_planner/planning_feedback_handler.h>

namespace l3_footstep_planning
{
/**
 * @brief A class defining a footstep planner environment for humanoid
 * robots used by the SBPL to perform planning tasks.
 *
 * The environment keeps track of all the planning states expanded during
 * the search. Each planning state can be accessed via its ID. Furthermore
 */
class FootstepPlannerEnvironment : public DiscreteSpaceInformation
{
public:
  // typedefs
  typedef l3::SharedPtr<FootstepPlannerEnvironment> Ptr;
  typedef l3::SharedPtr<const FootstepPlannerEnvironment> ConstPtr;

  FootstepPlannerEnvironment(const EnvironmentParameters& params, PlanningFeedbackHandler& feedback_handler);

  inline const StateSpace::Ptr& getStateSpace() { return state_space_; }

  inline int getNumExpandedStates() const { return num_expanded_states_; }

  inline int getNumVisitedStates() const { return num_visited_states_; }

  void reset();

  inline void stateExpanded(PlanningState::ConstPtr state)
  {
    num_expanded_states_++;
    feedback_handler_.stateExpanded(state);
  }

  inline void stateVisited(PlanningState::ConstPtr state)
  {
    num_visited_states_++;
    feedback_handler_.stateVisited(state);
  }

  /**
   * @return The costs (in mm, truncated as int) to reach the
   * planning state ToStateID from within planning state FromStateID.
   */
  int GetFromToHeuristic(int FromStateID, int ToStateID) override;

  /**
   * @return The heuristic value to reach the start from within
   * the planning state stateID. (Used for backward planning.)
   */
  int GetStartHeuristic(int stateID) override;

  /**
   * @return The heuristic value to reach the goal from within the
   * planning state stateID (used for forward planning).
   */
  int GetGoalHeuristic(int stateID) override;

  /**
   * @brief Calculates the successor states and the corresponding costs
   * when performing the footstep set on the planning state SourceStateID.
   * (Used for forward planning.)
   */
  void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV) override;

  /**
   * @brief Calculates the predecessor states and the corresponding costs
   * when reversing the footstep set on the planning state TargetStateID.
   * (Used for backward planning.)
   */
  void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV) override;

  /**
   * @brief Used for RStar: generate succs/preds at some
   * domain-dependent distance. The number of generated succs/preds is up
   * to the environment.
   */
  void GetRandomPredsatDistance(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CLowV) override;

  /**
   * @brief Used for RStar: generate succs/preds at some
   * domain-dependent distance. The number of generated succs/preds is up
   * to the environment.
   */
  void GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV) override;

  /// @return True if two states meet the same condition. Used for R*.
  bool AreEquivalent(int StateID1, int StateID2) override;

  bool InitializeEnv(const char* sEnvFile) override;

  bool InitializeMDPCfg(MDPConfig* MDPCfg) override;

  void PrintEnv_Config(FILE* fOut) override;

  void PrintState(int stateID, bool bVerbose, FILE* fOut) override;

  void SetAllActionsandAllOutcomes(CMDPSTATE* state) override;

  void SetAllPreds(CMDPSTATE* state) override;

  int SizeofCreatedEnv() override { return StateSpaceManager::getNumStates(); }

protected:
  /**
   * @return The costs (in mm, truncated as int) to reach the
   * planning state ToStateID from within planning state FromStateID.
   */
  int GetFromToHeuristic(PlanningState::ConstPtr from, PlanningState::ConstPtr to, PlanningState::ConstPtr start, PlanningState::ConstPtr goal);

  const EnvironmentParameters& params_;

  StateSpace::Ptr state_space_;

  PlanningFeedbackHandler& feedback_handler_;

  size_t num_expanded_states_;
  size_t num_visited_states_;
};
}  // namespace l3_footstep_planning

#endif
