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

#ifndef L3_PLANNING_VIS_PLUGIN_H__
#define L3_PLANNING_VIS_PLUGIN_H__

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include <vigir_pluginlib/plugin.h>

#include <l3_footstep_planning_libs/modeling/step_plan.h>
#include <l3_footstep_planning_libs/helper.h>

namespace l3_footstep_planning
{
class PlanningVisPlugin : public vigir_pluginlib::Plugin
{
public:
  // typedefs
  typedef l3::SharedPtr<PlanningVisPlugin> Ptr;
  typedef l3::SharedPtr<const PlanningVisPlugin> ConstPtr;

  PlanningVisPlugin(const std::string& name);

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  virtual void clear() {}

  /**
   * @note All input data is expressed in planner frame, hence each foothold pose represents
   * the center of the sole.
   */
  virtual void visualize(const msgs::StepPlanRequest& step_plan_request) {}
  virtual void visualize(const msgs::PlanningFeedback& planning_feedback) {}

  virtual void visualize(const StepPlan& step_plan);
  virtual void visualize(const Step& step) {}

  virtual visualization_msgs::MarkerArray getMarkerArray() const { return visualization_msgs::MarkerArray(); }

  /**
   * @brief This function is called when the collected visualization should be published.
   */
  virtual void trigger() {}

protected:
  inline bool ignoreFootIdx(const FootIndex& foot_idx) const { return !foot_idx_whitelist_.empty() && foot_idx_whitelist_.find(foot_idx) == foot_idx_whitelist_.end(); }

  /**
   * @brief Filters a given container so that only elements with a foot idx from the whitelist
   * are remaining.
   * @param Array Containter type
   * @return Filtered container consisting of elements according to the whitelist
   */
  template <typename Array>
  inline Array applyFootIdxWhitelist(const Array& array) const { return l3_footstep_planning::applyFootIdxWhitelist(array, foot_idx_whitelist_); }

  inline const FootIndexSet& footIdxWhitelist() const { return foot_idx_whitelist_; }

private:
  FootIndexSet foot_idx_whitelist_; // if not empty, do only consider given ones (whitelist)
};
}  // namespace l3_footstep_planning

#endif
