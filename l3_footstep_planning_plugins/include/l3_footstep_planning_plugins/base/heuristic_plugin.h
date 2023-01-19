//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_HEURISTIC_PLUGIN_H__
#define L3_FOOTSTEP_PLANNING_HEURISTIC_PLUGIN_H__

#include <ros/ros.h>

#include <l3_footstep_planning_plugins/base/footstep_planning_plugin.h>

#include <l3_footstep_planning_libs/modeling/state.h>

namespace l3_footstep_planning
{
class HeuristicPlugin : public virtual FootstepPlanningPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<HeuristicPlugin> Ptr;
  typedef l3::SharedPtr<const HeuristicPlugin> ConstPtr;

  HeuristicPlugin(const std::string& name);

  bool isUnique() const final { return false; }

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  /**
   * @brief Computes heuristic value between "from" and "to" states. The start and goal states
   * may be considered in computation.
   * @param from Beginning state to compute the heuristic from
   * @param to Target state to compute the heuristic to
   * @param start Start state of planning request
   * @param goal Goal state of planning request
   * @return Computed heuristic value between both states
   */
  virtual double getHeuristicValue(const State& from, const State& to, const State& start, const State& goal) const;

  double getWeight() const { return weight_; }

protected:
  /**
   * @brief Computes heuristic value between "from" and "to" footholds. The start and goal states
   * may be considered in computation.
   * @param from Beginning foothold to compute the heuristic from
   * @param to Target foothold to compute the heuristic to
   * @param start Start state of planning request
   * @param goal Goal state of planning request
   * @return Computed heuristic value between both footholds
   */
  virtual double getHeuristicValue(const Foothold& from, const Foothold& to, const State& start, const State& goal) const { return 0.0; }

  /**
   * @brief Computes heuristic value between "from" and "to" floating bases. The start and goal states
   * may be considered in computation.
   * @param from Beginning floating base to compute the heuristic from
   * @param to Target floating base to compute the heuristic to
   * @param start Start state of planning request
   * @param goal Goal state of planning request
   * @return Computed heuristic value between both footholds
   */
  virtual double getHeuristicValue(const FloatingBase& from, const FloatingBase& to, const State& start, const State& goal) const { return 0.0; }

  /**
   * @brief Detrermines the final heuristic value by dividing by either the number of footholds of state or the
   * number of explicitely defined foot_idx.
   * @param state State to compute transition heuristic from
   * @param h_val Current heuristic value
   * @return Normalize heuristic value
   */
  double normalizeResult(const State& state, double h_val) const;

  double weight_;

  double max_heuristic_value_;
};
}  // namespace l3_footstep_planning

#endif
