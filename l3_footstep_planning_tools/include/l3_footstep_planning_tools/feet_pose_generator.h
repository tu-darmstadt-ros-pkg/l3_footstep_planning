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

#ifndef L3_FEET_POSE_GENERATOR_H__
#define L3_FEET_POSE_GENERATOR_H__

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

#include <l3_terrain_model/terrain_model.h>

namespace l3_footstep_planning
{
class FeetPoseGenerator
{
public:
  // typedefs
  typedef l3::SharedPtr<FeetPoseGenerator> Ptr;
  typedef l3::SharedPtr<const FeetPoseGenerator> ConstPtr;

  FeetPoseGenerator(ros::NodeHandle& nh);

  void setRobotPose(const geometry_msgs::PoseStampedConstPtr& robot_pose);
  void setRobotPoseWithCovariance(const geometry_msgs::PoseWithCovarianceStampedConstPtr& robot_pose);

  msgs::ErrorStatus generateFeetPose(const msgs::FeetPoseRequest& request, msgs::FootholdArray& feet);

  void setTerrainModel(l3_terrain_modeling::TerrainModelMsg::ConstPtr terrain_model);
  void setGridMap(grid_map_msgs::GridMap::ConstPtr grid_map);

protected:

  msgs::ErrorStatus updateFeetPose(msgs::FootholdArray& feet);

  bool getCurrentFeetPose(msgs::FootholdArray& feet, const std::string& request_frame);

  /**
   * This method assumes that the given pose is in center between both feet.
   */
  msgs::FootholdArray generateFeetPose(const geometry_msgs::PoseStamped& pose);

  void pelvisToGroundTransform(msgs::FootholdArray& feet) const;

  l3_terrain_modeling::TerrainModel::Ptr terrain_model_;

  bool has_robot_pose_;
  geometry_msgs::PoseStamped robot_pose_;

  tf::TransformListener tf_listener_;
};
}  // namespace l3_footstep_planning

#endif
