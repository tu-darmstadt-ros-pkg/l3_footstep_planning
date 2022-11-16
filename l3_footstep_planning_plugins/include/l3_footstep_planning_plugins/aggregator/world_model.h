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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_WORLD_MODEL_H__
#define L3_FOOTSTEP_PLANNING_PLUGINS_WORLD_MODEL_H__

#include <ros/ros.h>

#include <l3_footstep_planning_plugins/aggregator/extended_plugin_aggregator.h>
#include <l3_footstep_planning_plugins/base/world_model_plugin.h>
#include <l3_footstep_planning_plugins/base/terrain_model_plugin.h>

namespace l3_footstep_planning
{
class WorldModel : public ExtendedPluginAggregator<WorldModel, WorldModelPlugin>
{
public:
  // typedefs
  typedef std::list<SharedLock> ModelLocks;

  WorldModel();

  bool loadPlugins(bool print_warning = true) override;
  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  void resetPlugins() override;

  ModelLocks lockModel() const;

  // evaluation will be done in alphabetical order of plugin names
  bool isAccessible(const Foothold& foothold) const;
  bool isAccessible(const FloatingBase& floating_base) const;
  bool isAccessible(const State& state) const;
  bool isAccessible(const PlanningState& state) const;

  void useTerrainModel(bool enabled);
  bool isTerrainModelAvailable() const;
  TerrainModelPlugin::ConstPtr getTerrainModel() const;

  /**
   * @brief update z, roll and pitch of state based on terrain model
   * @return false if terrain model is available but has no data for given state,
   *         otherwise true even if no terrain model is available
   **/
  TerrainResult getHeight(double x, double y, double& height) const;
  TerrainResult update3DData(Foothold& foothold) const;

protected:
  bool use_terrain_model_;
  TerrainModelPlugin::Ptr terrain_model_plugin_;
};
}  // namespace l3_footstep_planning

typedef l3_footstep_planning::WorldModel PlannerWorldModel;

#endif
