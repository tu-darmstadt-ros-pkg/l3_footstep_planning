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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_STATE_GENERATOR_H__
#define L3_FOOTSTEP_PLANNING_PLUGINS_STATE_GENERATOR_H__

#include <random>

#include <ros/ros.h>

#include <l3_footstep_planning_libs/threading/threading_manager.h>

#include <l3_footstep_planning_plugins/aggregator/extended_plugin_aggregator.h>
#include <l3_footstep_planning_plugins/base/state_generator_plugin.h>

#include <l3_footstep_planning_plugins/std/state_generator/expand_state_job.h>

namespace l3_footstep_planning
{
struct PlanningStateGenResult
{
  PlanningStateGenResult()
    : state()
    , cost(0.0)
    , risk(0.0)
  {}

  PlanningStateGenResult(PlanningStateHashed::ConstPtr state, double cost = 0.0, double risk = 0.0)
    : state(state)
    , cost(cost)
    , risk(risk)
  {}

  PlanningStateHashed::ConstPtr state;
  double cost;
  double risk;
};

class StateGenerator : public ExtendedPluginAggregator<StateGenerator, StateGeneratorPlugin>
{
public:
  StateGenerator();

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  /**
   * @brief Generates list of valid adjacent predecessor states.
   * Therefore all states must be completely derived and cross-checked including:
   * - reachability (Reachability)
   * - 3D pose (WorldModel)
   * - collision checks (WorldModel)
   * - cost and risk (StepCostEstimator)
   * - post processing (PostProcessor)
   * @param state Current state from which all valid adjacent states should be determined.
   * @param start Start state of planning request
   * @param goal Goal state of planning request
   * @return List of all valid adjacent states
   */
  inline std::list<PlanningStateGenResult> generatePredecessors(PlanningState::ConstPtr state, State::ConstPtr start, State::ConstPtr goal) const
  {
    return processJobs(generatePredecessorsJobs(state, start, goal));
  }

  /**
   * @brief Generates list of valid adjacent successor states.
   * Therefore all states must be completely derived and cross-checked including:
   * - reachability (Reachability)
   * - 3D pose (WorldModel)
   * - collision checks (WorldModel)
   * - cost and risk (StepCostEstimator)
   * - post processing (PostProcessor)
   * @param state Current state from which all valid adjacent states should be determined.
   * @param start Start state of planning request
   * @param goal Goal state of planning request
   * @return List of all valid adjacent states
   */
  std::list<PlanningStateGenResult> generateSuccessors(PlanningState::ConstPtr state, State::ConstPtr start, State::ConstPtr goal) const
  {
    return processJobs(generateSuccessorsJobs(state, start, goal));
  }

  /**
   * @brief Generates list of randomly selected valid adjacent predecessor states.
   * Therefore all states must be completely derived and cross-checked including:
   * - reachability (Reachability)
   * - 3D pose (WorldModel)
   * - collision checks (WorldModel)
   * - post processing (PostProcessor)
   * @param state Current state from which all valid adjacent states should be determined.
   * @param num Number of random samples to select
   * @param start Start state of planning request
   * @param goal Goal state of planning request
   * @return List of random valid adjacent states
   */
  std::list<PlanningStateGenResult> generateRandomPredecessors(PlanningState::ConstPtr state, unsigned int num, State::ConstPtr start, State::ConstPtr goal) const
  {
    return processJobs(selectRandomJobs(generatePredecessorsJobs(state, start, goal, true), num));
  }

  /**
   * @brief Generates list of randomly selected valid adjacent successor states.
   * Therefore all states must be completely derived and cross-checked including:
   * - reachability (Reachability)
   * - 3D pose (WorldModel)
   * - collision checks (WorldModel)
   * - post processing (PostProcessor)
   * @param state Current state from which all valid adjacent states should be determined.
   * @param num Number of random samples to select
   * @param start Start state of planning request
   * @param goal Goal state of planning request
   * @return List of random valid adjacent states
   */
  std::list<PlanningStateGenResult> generateRandomSuccessors(PlanningState::ConstPtr state, unsigned int num, State::ConstPtr start, State::ConstPtr goal) const
  {
    return processJobs(selectRandomJobs(generateSuccessorsJobs(state, start, goal, true), num));
  }

  /**
   * @brief Generates (intermediate) states in order to reach goal. Only used for forward planning.
   * @param state Current state
   * @param start Start planning state
   * @param goal Goal planning state
   * @return List of possible (intermediate) predecessor states
   */
  std::list<PlanningStateGenResult> generateNearStartStates(PlanningState::ConstPtr current, PlanningState::ConstPtr start, PlanningState::ConstPtr goal) const;

  /**
   * @brief Generates (intermediate) states in order to reach goal. Only used for forward planning.
   * @param state Current state
   * @param start Start planning state
   * @param goal Goal planning state
   * @return List of possible (intermediate) successor states
   */
  std::list<PlanningStateGenResult> generateNearGoalStates(PlanningState::ConstPtr current, PlanningState::ConstPtr start, PlanningState::ConstPtr goal) const;

protected:
  std::vector<threading::ExpandStateJob::Ptr> generatePredecessorsJobs(PlanningState::ConstPtr state, State::ConstPtr start, State::ConstPtr goal, bool lazy = false) const;
  std::vector<threading::ExpandStateJob::Ptr> generateSuccessorsJobs(PlanningState::ConstPtr state, State::ConstPtr start, State::ConstPtr goal, bool lazy = false) const;

  /**
   * @brief Selects randomly jobs from a given input list.
   * @param jobs Input list of jobs
   * @param num Number of randomly selected jobs
   * @return List of randomly selected jobs with the size of min(num, size(jobs))
   */
  std::vector<threading::ExpandStateJob::Ptr> selectRandomJobs(std::vector<threading::ExpandStateJob::Ptr> jobs, unsigned int num) const;

  /**
   * @brief Generates state generation jobs for planning states which are transitioning
   * from the current state to the target state using the given foot movement patterns.
   * @param current Current planning state
   * @param target Target planning state
   * @param start Start state of planning request
   * @param goal Goal state of planning request
   * @param patterns Leg movement patterns
   * @return Planning states generation jobs
   */
  std::vector<threading::ExpandStateJob::Ptr> generateNearStateJobs(PlanningState::ConstPtr current, PlanningState::ConstPtr target, State::ConstPtr start, State::ConstPtr goal,
                                                                    const ExpandStatesIdxArray& patterns) const;

  template <template <typename...> class Container>
  std::list<PlanningStateGenResult> processJobs(const Container<threading::ExpandStateJob::Ptr>& jobs) const
  {
    std::list<PlanningStateGenResult> result;

    expand_states_manager_->addJobs(jobs);
    expand_states_manager_->waitUntilJobsFinished();

    for (threading::ExpandStateJob::ConstPtr job : jobs)
    {
      if (job->successful)
      {
        PlanningStateGenResult r;
        r.state = job->next;
        r.cost = job->cost;
        r.risk = job->risk;
        result.push_back(r);
      }
    }

    return result;
  }

  FootIndexArray removeIndirectIdx(FootIndexArray foot_idx) const;

  mutable threading::ThreadingManager<threading::ExpandStateJob>::Ptr expand_states_manager_;

  mutable std::random_device random_device_;  // will be used to obtain a seed for the random number engine
  mutable std::mt19937 random_generator_;     // standard mersenne_twister_engine seeded with rd()

  FootIndexSet indirect_foot_idx_;  // indirect links are not automatically generated by the default state generator
};
}  // namespace l3_footstep_planning

#endif
