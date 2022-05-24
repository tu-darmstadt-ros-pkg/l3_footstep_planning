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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_POLYGONAL_STATE_GENERATOR_H__
#define L3_FOOTSTEP_PLANNING_PLUGINS_POLYGONAL_STATE_GENERATOR_H__

#include <l3_footstep_planning_libs/modeling/footstep.h>

#include <l3_footstep_planning_plugins/base/state_generator_plugin.h>

namespace l3_footstep_planning
{
class PolygonalStateGenerator : public StateGeneratorPlugin
{
protected:
  typedef std::pair<const FootIndex, std::vector<Footstep>> FootstepSetPair;
  typedef std::map<FootIndex, std::vector<Footstep>> FootstepSetMap;

public:
  PolygonalStateGenerator(const std::string& name = "polygonal_state_generator");

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  std::list<StateGenResult> generatePredStateResults(const PlanningState& state, const State& start, const State& goal, const ExpandStatesIdx& state_expansion_idx) const override;
  std::list<StateGenResult> generateSuccStateResults(const PlanningState& state, const State& start, const State& goal, const ExpandStatesIdx& state_expansion_idx) const override;

protected:
  std::list<StateGenResult> generatePredFootholds(const PlanningState& state, const ExpandStatesIdx& state_expansion_idx, const FootstepSetMap& footsteps) const;
  std::list<StateGenResult> generateSuccFootholds(const PlanningState& state, const ExpandStatesIdx& state_expansion_idx, const FootstepSetMap& footsteps) const;

  // The set of footsteps used.
  FootstepSetMap footstep_sets_;
};
}  // namespace l3_footstep_planning

#endif
