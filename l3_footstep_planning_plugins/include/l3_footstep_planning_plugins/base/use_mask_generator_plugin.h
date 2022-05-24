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

#ifndef L3_USE_MASK_GENERATOR_PLUGIN_H__
#define L3_USE_MASK_GENERATOR_PLUGIN_H__

#include <vigir_pluginlib/plugin.h>

#include <l3_footstep_planning_libs/modeling/planning_state.h>

namespace l3_footstep_planning
{
enum USE_MASK_BITS
{
  NO_USE = 0u,
  USE_ALWAYS = 1u
};

typedef unsigned int UseMask;

struct UseMaskSuperimposition
{
  UseMaskSuperimposition(const UseMask& use_mask, double weight = 1.0)
    : use_mask(use_mask)
    , weight(weight)
  {}

  UseMaskSuperimposition()
    : use_mask(NO_USE)
    , weight(0.0)
  {}

  UseMask use_mask;
  double weight;
};

class UseMaskGeneratorPlugin : public virtual vigir_pluginlib::Plugin
{
public:
  // typedefs
  typedef l3::SharedPtr<UseMaskGeneratorPlugin> Ptr;
  typedef l3::SharedPtr<const UseMaskGeneratorPlugin> ConstPtr;

  UseMaskGeneratorPlugin(const std::string& name);

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  virtual void reset() {}

  /**
   * @brief Determines use mask to apply for specific state generator plugins to be used.
   * @param state Current planning state
   * @param start Start planning state
   * @param goal goal planning state
   * @return use mask which selects the plugins to be used
   */
  virtual UseMask determineStateGenerationUseMask(const PlanningState& state, const State& start, const State& goal) const { return getNegUseMask(); }

  /**
   * @brief Determines use mask to apply for specific step cost estimator plugins to be used.
   * @param state Current planning state
   * @param start Start planning state
   * @param goal goal planning state
   * @return use mask which selects the plugins to be used
   */
  virtual UseMask determineStepCostEstimatorUseMask(const PlanningState& state, const State& start, const State& goal, std::list<UseMaskSuperimposition>& masks) const
  {
    return getNegUseMask();
  }

  /**
   * @brief Determines use mask to apply for specific heuristic plugins to be used.
   * @param from Current state
   * @param to Target state
   * @param start Start planning state
   * @param goal goal planning state
   * @return use mask which selects the plugins to be used
   */
  virtual UseMask determineHeuristicUseMask(const State& from, const State& to, const State& start, const State& goal, std::list<UseMaskSuperimposition>& masks) const
  {
    return getNegUseMask();
  }

protected:
  inline const UseMask& getPosUseMask() const { return pos_use_mask_; }
  inline const UseMask& getNegUseMask() const { return neg_use_mask_; }

private:
  UseMask pos_use_mask_ = NO_USE;
  UseMask neg_use_mask_ = NO_USE;
};
}  // namespace l3_footstep_planning

#endif
