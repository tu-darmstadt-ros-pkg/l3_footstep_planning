//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, Filip Bjelonic, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_HFS_HEURISTIC_H__
#define L3_FOOTSTEP_PLANNING_PLUGINS_HFS_HEURISTIC_H__

#include <ros/ros.h>

#include <l3_libs/types/types.h>

#include <l3_footstep_planning_plugins/aggregator/extended_plugin_aggregator.h>
#include <l3_footstep_planning_plugins/base/hfs_heuristic_plugin.h>

namespace l3_footstep_planning
{
using namespace l3;

class HFSHeuristic : public ExtendedPluginAggregator<HFSHeuristic, HFSHeuristicPlugin>
{
public:
  HFSHeuristic();

  /**
   * @brief Gives you the index of the foot in FootIndexArray idcs which is hardest to plan next. If it is empty, the base should be planned.
   * @param next The next foot index which should be planned by A* and where states should be expanded
   * @param idxs From the current cycle the footsteps that are missing to be planned to complete the cycle
   * @param base_planned If True, the heuristic for the base shall be compared to the foot heuristic and decided, if the base should be planned next
   * @param step the current step of the robot from which the next states will be expanded
   * @return True if it worked, false if something went wrong. Handle the false case carefully to avoid errors.
   */
  bool getHardest(l3::ExpandStatesIdx& next, const l3::ExpandStatesIdx& idxs, Step::ConstPtr step) const;
};
}  // namespace l3_footstep_planning

#endif
