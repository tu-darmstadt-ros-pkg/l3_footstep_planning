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

#ifndef L3_FOOTSTEP_PLANNING_STEP_COST_ESTIMATOR_PLUGIN_H__
#define L3_FOOTSTEP_PLANNING_STEP_COST_ESTIMATOR_PLUGIN_H__

#include <ros/ros.h>

#include <l3_footstep_planning_plugins/base/footstep_planning_plugin.h>

#include <l3_footstep_planning_libs/modeling/planning_state.h>

namespace l3_footstep_planning
{
class StepCostEstimatorPlugin : public virtual FootstepPlanningPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<StepCostEstimatorPlugin> Ptr;
  typedef l3::SharedPtr<const StepCostEstimatorPlugin> ConstPtr;

  StepCostEstimatorPlugin(const std::string& name);

  bool isUnique() const final { return false; }

  /**
   * @brief Returns cost of transition represented by the planning state. Hereby, for each step data
   * the resulting cost is evaluated by calling the getCost(step_data, ...). The default implementation
   * sums up the final cost and risk.
   * @param state State to compute transition cost from
   * @param cost [out] Calculated cost
   * @param cost_multiplier [out] Cost multiplication factor (must be >= 1.0; default: 1.0)
   * @param risk [out] Calculated risk
   * @param risk_multiplier [out] Risk multiplication factor (must be >= 1.0; default: 1.0)
   * @return
   */
  virtual bool getCost(const PlanningState& state, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const;

protected:
  /**
   * @brief Returns cost of transition represented by the step data.
   * @param step_data StepData to compute transition cost from
   * @param cost [out] Calculated cost
   * @param cost_multiplier [out] Cost multiplication factor (must be >= 1.0; default: 1.0)
   * @param risk [out] Calculated risk
   * @param risk_multiplier [out] Risk multiplication factor (must be >= 1.0; default: 1.0)
   * @return
   */
  virtual bool getCost(const FootStepData& step_data, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const;

  /**
   * @brief Returns cost of transition represented by the floating base step data.
   * @param base_step_data BaseStepData to compute transition cost from
   * @param cost [out] Calculated cost
   * @param cost_multiplier [out] Cost multiplication factor (must be >= 1.0; default: 1.0)
   * @param risk [out] Calculated risk
   * @param risk_multiplier [out] Risk multiplication factor (must be >= 1.0; default: 1.0)
   * @return
   */
  virtual bool getCost(const BaseStepData& base_step_data, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const;

  /**
   * @brief Detrermines the final cost and risk by dividing by either the number of footholds of state or the
   * number of explicitely defined foot_idx.
   * @param state State to compute transition cost from
   * @param cost [out] Calculated cost
   * @param risk [out] Calculated risk
   */
  void normalizeResult(const PlanningState& state, double& cost, double& risk) const;
};
}  // namespace l3_footstep_planning

#endif
