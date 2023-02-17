//=================================================================================================
// Copyright (c) 2023, Simon Giegerich, Alexander Stumpf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_FAST_MARCHING_METHOD_HEURISTIC_H
#define L3_FOOTSTEP_PLANNING_PLUGINS_FAST_MARCHING_METHOD_HEURISTIC_H

#include <l3_footstep_planning_plugins/base/hlut_heuristic_plugin.h>

namespace l3_footstep_planning
{
/**
 * @brief Determining the heuristic value using the Fast Marching Method.
 *
 * @param resolution The resolution of the HLUT.
 * @param size_x The size of the HLUT in x-direction.
 * @param size_y The size of the HLUT in y-direction.
 * @param check_foothold_accessibility Specifies whether the foothold of the robot should be used for the accessibility check.
 * @param check_floating_base_accessibility Specifies whether the floating base of the robot (and its projection to the ground) should be used for the accessibility check.
 * @param visualize Specifies whether the HLUT should be visualized.
 * @param vis_frame_id The frame id of the HLUT visualization.
 * @param vis_topic The topic of the HLUT visualization.
 * @param vis_layer The layer of the HLUT visualization.
 */
class FastMarchingMethodHeuristic : public HLUTHeuristicPlugin
{
public:
  /**
   * @brief Default constructor.
   */
  FastMarchingMethodHeuristic();

protected:
  std::vector<l3::PositionIndex> getNeighbors(const l3::PositionIndex& current_index) const override;

  hlutEntry computeHLUTEntryOfNeighbor(const l3::PositionIndex& neighbor, const hlutEntry& current_entry) const override;

private:
  const float inf = std::numeric_limits<float>::infinity();
};
}  // namespace l3_footstep_planning

#endif  // L3_FOOTSTEP_PLANNING_PLUGINS_FAST_MARCHING_METHOD_HEURISTIC_H
