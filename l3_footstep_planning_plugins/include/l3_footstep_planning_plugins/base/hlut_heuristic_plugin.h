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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_HLUT_HEURISTIC_PLUGIN_H
#define L3_FOOTSTEP_PLANNING_PLUGINS_HLUT_HEURISTIC_PLUGIN_H

#include <queue>

#include <grid_map_ros/grid_map_ros.hpp>

#include <l3_libs/conversions/l3_msg_conversions.h>
#include <l3_libs/robot_description/base_info.h>

#include <l3_footstep_planning_libs/types/heuristic_lookup_table.h>

#include <l3_footstep_planning_plugins/aggregator/world_model.h>
#include <l3_footstep_planning_plugins/base/heuristic_plugin.h>

namespace std
{
template <>
struct hash<l3::PositionIndex>
{
  size_t operator()(const l3::PositionIndex& index) const
  {
    // cantor pairing function
    return ((index.x() + index.y()) * (index.x() + index.y() + 1) >> 1) + index.y();
  }
};
}  // namespace std

namespace l3_footstep_planning
{
/**
 * @brief Base class for heuristic plugins that use a heuristic lookup table.
 *
 * @param resolution The resolution of the HLUT.
 * @param size_x The size of the HLUT in x-direction.
 * @param size_y The size of the HLUT in y-direction.
 * @param check_foothold_accessibility Specifies whether the foothold of the robot should be used for the accessibility check.
 * @param check_floating_base_accessibility Specifies whether the floating base of the robot should be used for the accessibility check.
 * @param visualize Specifies whether the HLUT should be visualized.
 * @param vis_frame_id The frame id of the HLUT visualization.
 * @param vis_topic The topic of the HLUT visualization.
 * @param vis_layer The layer of the HLUT visualization.
 */
class HLUTHeuristicPlugin : public HeuristicPlugin
{
public:
  /**
   * @brief Default constructor.
   */
  HLUTHeuristicPlugin(const std::string& name);

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

  void preparePlanning(const msgs::StepPlanRequest& req) override;

  double getHeuristicValue(const State& from, const State& to, const State& start, const State& goal) const override;

  double getHeuristicValue(const Foothold& from, const Foothold& to, const State& start, const State& goal) const override;

  double getHeuristicValue(const FloatingBase& from, const FloatingBase& to, const State& start, const State& goal) const override;

protected:
  /**
   * @brief An entry of HLUT consisting of its index and heuristic value.
   */
  struct hlutEntry
  {
    // The index of the entry.
    l3::PositionIndex index;

    // The heuristic value of the entry.
    float heuristic_distance;
  };

  /**
   * @brief A comparator for the priority queue of the HLUT computation.
   */
  struct gridMapHeuristicEntryCompare
  {
    bool operator()(const hlutEntry& a, const hlutEntry& b) { return a.heuristic_distance > b.heuristic_distance; }
  };

  // The heuristic lookup table.
  HeuristicLookupTable hlut_;

  // The resolution of the HLUT.
  l3::DiscreteResolution resolution_;

  // The size of the HLUT in x-direction.
  int size_x_;

  // The size of the HLUT in y-direction.
  int size_y_;

  // Specifies whether the foothold of the robot should be used for the accessibility check.
  bool check_foothold_accessibility_;

  // Specifies whether the floating base of the robot should be used for the accessibility check.
  bool check_floating_base_accessibility_;

  // The vector containing the yaw angles to be considered for the accessibility check.
  std::vector<double> angle_bins_;

  // The floating base of the start state.
  FloatingBase start_fb_;

  // The position of the start state.
  l3::Position2D start_pos_;

  // The floating base of the goal state.
  FloatingBase goal_fb_;

  // The position of the goal state.
  l3::Position2D goal_pos_;

  // Specifies whether the HLUT should be visualized.
  bool visualize_;

  // The frame id of the HLUT visualization.
  std::string vis_frame_id_;

  // The topic of the HLUT visualization.
  std::string vis_topic_;

  // The layer of the HLUT visualization.
  std::string vis_layer_;

  // The publisher of the HLUT visualization.
  ros::Publisher hlut_pub_;

  // Cache for HLUT index accessibility
  mutable std::unordered_map<l3::PositionIndex, bool> is_accessible_;

  /**
   * @brief Initializes the HLUT.
   * @param start_fb The start floating base (used to determine the HLUT size).
   * @param goal_fb The goal floating base (used to determine the HLUT size).
   * @param goal_pos The goal position (the center of the HLUT).
   * @return The initialized HLUT.
   */
  virtual HeuristicLookupTable initializeHLUT(const FloatingBase& start_fb, const FloatingBase& goal_fb, const l3::Position2D& goal_pos) const;

  /**
   * @brief Get the neighbor indices of the specified index.
   * @param current_index The index for which the neighbors should be computed.
   * @return The neighbor indices of the specified index.
   */
  virtual std::vector<l3::PositionIndex> getNeighbors(const l3::PositionIndex& current_index) const { return {}; }

  /**
   * @brief Get the valid neighbor indices of the specified neighbor indices.
   * @param neighbors The neighbor indices for which the valid ones should be computed.
   * @return The valid neighbor indices.
   */
  virtual std::vector<l3::PositionIndex> getValidNeighbors(const std::vector<l3::PositionIndex>& neighbors) const;

  /**
   * @brief Computes the HLUT entry of the specified neighbor.
   * @param neighbor The neighbor for which the HLUT entry should be computed.
   * @param current_index The current index.
   * @return The computed HLUT entry.
   */
  virtual hlutEntry computeHLUTEntryOfNeighbor(const l3::PositionIndex& neighbor, const hlutEntry& current_entry) const { return { neighbor, 0.0 }; }

  /**
   * @brief Publishes the HLUT as a grid map.
   */
  virtual void visualizeHLUT() const;
};
}  // namespace l3_footstep_planning

#endif  // L3_FOOTSTEP_PLANNING_PLUGINS_HLUT_HEURISTIC_PLUGIN_H
