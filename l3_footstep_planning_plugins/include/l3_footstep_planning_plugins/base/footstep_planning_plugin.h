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

#ifndef L3_FOOTSTEP_PLANNING_PLUGIN_H__
#define L3_FOOTSTEP_PLANNING_PLUGIN_H__

#include <vigir_pluginlib/plugin.h>

#include <l3_footstep_planning_libs/helper.h>

#include <l3_footstep_planning_plugins/base/use_mask_generator_plugin.h>

namespace l3_footstep_planning
{
class FootstepPlanningPlugin : public virtual vigir_pluginlib::Plugin
{
public:
  typedef l3::SharedPtr<FootstepPlanningPlugin> Ptr;
  typedef l3::SharedPtr<const FootstepPlanningPlugin> ConstPtr;

  FootstepPlanningPlugin(const std::string& name = std::string());

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  virtual void reset() {}

  inline const UseMask& getUseMask() const { return use_mask_; }
  inline bool canUse(const UseMask& use_mask) const { return static_cast<bool>(use_mask_ & use_mask); }

protected:
  inline void setUseMask(const UseMask& use_mask) { use_mask_ = use_mask; }

  inline bool ignoreFootIdx(const FootIndex& foot_idx) const { return !foot_idx_whitelist_.empty() && foot_idx_whitelist_.find(foot_idx) == foot_idx_whitelist_.end(); }

  inline FootIndexArray applyFootIdxWhitelist(const FootIndexArray& foot_idx) const { return l3_footstep_planning::applyFootIdxWhitelist(foot_idx, foot_idx_whitelist_); }
  inline FootholdArray applyFootIdxWhitelist(const FootholdArray& footholds) const { return l3_footstep_planning::applyFootIdxWhitelist(footholds, foot_idx_whitelist_); }
  inline FootholdPtrArray applyFootIdxWhitelist(const FootholdPtrArray& footholds) const { return l3_footstep_planning::applyFootIdxWhitelist(footholds, foot_idx_whitelist_); }
  inline FootholdConstPtrArray applyFootIdxWhitelist(const FootholdConstPtrArray& footholds) const { return l3_footstep_planning::applyFootIdxWhitelist(footholds, foot_idx_whitelist_); }

  inline const FootIndexSet& footIdxWhitelist() const { return foot_idx_whitelist_; }
  inline const FootIndexSet& indirectFootIdx() const { return indirect_foot_idx_; }

  inline bool forwardSearch() const { return forward_search_; }

private:
  UseMask use_mask_ = USE_ALWAYS;

  FootIndexSet foot_idx_whitelist_; // if not empty, do only consider given ones (whitelist)
  FootIndexSet indirect_foot_idx_;  // indirect footholds are automatically blacklisted and must be whitelistes explicitly

  bool forward_search_;
};
}  // namespace l3_footstep_planning

#endif
