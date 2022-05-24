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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_OCCUPANCY_GRID_MAP_HEURISTIC_H__
#define L3_FOOTSTEP_PLANNING_PLUGINS_OCCUPANCY_GRID_MAP_HEURISTIC_H__

#include <boost/thread/mutex.hpp>

#include <nav_msgs/OccupancyGrid.h>

#include <l3_footstep_planning_plugins/base/heuristic_plugin.h>
#include <l3_footstep_planning_plugins/std/world_model/grid_map_2d.h>

namespace l3_footstep_planning
{
class OccupancyGridMapHeuristic : public HeuristicPlugin
{
public:
  OccupancyGridMapHeuristic();

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;
  bool initialize(const vigir_generic_params::ParameterSet& params) override;

  double getHeuristicValue(const Foothold& from, const Foothold& to, const State& start, const State& goal) const override;

protected:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map);

  // subscribers
  ros::Subscriber occupancy_grid_map_sub_;

  // mutex
  mutable Mutex grid_map_shared_mutex_;

  // grid map data
  std::string grid_map_topic_;
  l3_gridmap_2d::GridMap2D distance_map_;
};
}  // namespace l3_footstep_planning

#endif
