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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_TRAVERSABILITY_MAP_MODEL_H
#define L3_FOOTSTEP_PLANNING_PLUGINS_TRAVERSABILITY_MAP_MODEL_H

#include <grid_map_ros/grid_map_ros.hpp>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_plugins/base/world_model_plugin.h>

namespace l3_footstep_planning
{
/**
 * @brief The TraversabilityMapModel class provides a world model plugin that uses a traversability map to check the accessibility of positions.
 *
 * @param traversability_map_topic The topic where the traversability map is published.
 * @param traversability_map_layer The layer of the traversability map that is used for the accessibility check.
 * @param min_traversability_weight The weight that specifies how much the min traversability value inside the robots polygon projection influences the accessibility check.
 * @param mean_traversability_weight The weight that specifies how much the mean traversability value inside the robots polygon projection influences the accessibility check.
 * @param traversability_threshold The threshold that specifies the traversability value that is required for a position to be accessible.
 * @param unknown_traversability_value The value that is used for areas that are not covered by the traversability map.
 * @param check_foothold_accessibility Specifies whether the foothold of the robot should be used for the accessibility check.
 * @param check_floating_base_accessibility Specifies whether the floating base of the robot (and its projection to the ground) should be used for the accessibility check.
 * @param num_sampling_points_x The number of sampling points in x direction that are used for the accessibility check.
 * @param num_sampling_points_y The number of sampling points in y direction that are used for the accessibility check.
 */
class TraversabilityMapModel : public WorldModelPlugin
{
public:
  /**
   * @brief Default constructor.
   */
  TraversabilityMapModel();

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

  bool isAccessible(const Foothold& foothold) const override;

  bool isAccessible(const FloatingBase& floatingBase) const override;

protected:
  /**
   * @brief Sets a new traversability map constPtr.
   * @param traversability_map_new New traversability map constPtr.
   */
  void mapCallback(const grid_map_msgs::GridMapConstPtr& traversability_map_new);

  /**
   * @brief Checks whether the traversability values inside the given polygon are above the threshold using the specified weights.
   * @param position The center position of the polygon.
   * @param size The size of the polygon in x and y direction.
   * @param yaw The orientation of the polygon.
   * @return true if the traversability values inside the given polygon are above the threshold using the specified weights, false otherwise.
   */
  bool isPolygonAccessible(const Vector2& position, const Vector2& size, double yaw) const;

  /**
   * @brief Checks whether the traversability values inside the given polygon are above the threshold using the specified weights.
   * @param num_sampling_points_min The smaller number of sampling points in x and y direction. (Used for best loop nesting)
   * @param num_sampling_points_max The higher number of sampling points in x and y direction. (Used for best loop nesting)
   * @param bottom_left_corner Position of the bottom left corner of the polygon.
   * @param min_step_x The x value of the step vector corresponding to the smaller number of sampling points.
   * @param min_step_y The y value of the step vector corresponding to the smaller number of sampling points.
   * @param max_step_x The x value of the step vector corresponding to the higher number of sampling points.
   * @param max_step_y The y value of the step vector corresponding to the higher number of sampling points.
   * @return true if the traversability values inside the given polygon are equal or above the threshold using the specified weights, false otherwise.
   */
  bool iteratePolygon(int num_sampling_points_min, int num_sampling_points_max, l3::Position2D& bottom_left_corner, double min_step_x, double min_step_y, double max_step_x,
                      double max_step_y) const;

  // Subscriber for the traversability map
  ros::Subscriber traversability_map_sub_;

  // Traversability map topic
  std::string traversability_map_topic_;

  // Traversability map layer
  std::string traversability_map_layer_;

  // Traversability map
  grid_map::GridMap traversability_map_;

  // Weight of the min traversability for the accessibility check
  double min_traversability_weight_;

  // Weight of the mean traversability for the accessibility check
  double mean_traversability_weight_;

  // Traversability threshold until the position is considered accessible
  double traversability_threshold_;

  // The threshold that must be undercut by the min traversability so that the mean traversability becomes irrelevant
  double early_exit_threshold_;

  // Traversability value for areas with no information about the traversability
  float unknown_traversability_value_;

  // Specifies whether areas with no information about the traversability are considered accessible
  bool unknown_area_is_accessible_;

  // Specifies whether the foothold of the robot should be used for the accessibility check
  bool check_foothold_accessibility_;

  // Specifies whether the floating base of the robot should be used for the accessibility check
  bool check_floating_base_accessibility_;

  // Safety margin for the robot's upper body in x direction
  double upper_body_margin_x_;

  // Safety margin for the robot's upper body in y direction
  double upper_body_margin_y_;

  // Safety margin for the robot's foothold in x direction
  double foothold_margin_x_;

  // Safety margin for the robot's foothold in y direction
  double foothold_margin_y_;

  // Number of sampling points in x direction of the robot's upper body
  int num_sampling_points_x_;

  // Number of sampling points in y direction of the robot's upper body
  int num_sampling_points_y_;

  // The size of the robot's upper body
  Vector2 upper_body_size_;

  // The sizes of the robot's feet
  std::vector<Vector2> feet_sizes_;

  // teh shapes of the robot's feet
  std::vector<FootInfo::Shape> feet_shapes_;
};
}  // namespace l3_footstep_planning

#endif  // L3_FOOTSTEP_PLANNING_PLUGINS_TRAVERSABILITY_MAP_MODEL_H
