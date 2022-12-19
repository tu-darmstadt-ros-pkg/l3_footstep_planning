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

#ifndef L3_FEET_POSE_GENERATOR_CLIENT_H__
#define L3_FEET_POSE_GENERATOR_CLIENT_H__

#include <ros/ros.h>

#include <l3_libs/types/types.h>

#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

namespace l3_footstep_planning
{
using namespace l3;

/**
 * @brief The FeetPoseGeneratorClient interfaces the FeetPoseGeneratorNode to provide
 * simple access to basic functionality.
 */
class FeetPoseGeneratorClient
{
public:
  // typedefs
  typedef SharedPtr<FeetPoseGeneratorClient> Ptr;
  typedef SharedPtr<const FeetPoseGeneratorClient> ConstPtr;

  FeetPoseGeneratorClient(ros::NodeHandle& nh);

  /**
   * @brief Generates footholds in neutral stance at given position
   * @param footholds [out] Generated footholds
   * @param floating_base Floating base from which the footholds should be derived from
   * @param frame_id Target frame the footholds are expressed in
   * @return List of footholds representing the state at the given position
   */
  msgs::ErrorStatus getFootholds(msgs::FootholdArray& footholds, const FloatingBase& floating_base, const std::string& frame_id);
  msgs::ErrorStatus getFootholds(FootholdArray& footholds, const FloatingBase& floating_base, const std::string& frame_id);

  /**
   * @brief Generates footholds in neutral stance at given position
   * @param footholds [out] Generated footholds
   * @param feet_center Geometric feet center from which the footholds should be derived from
   * @param frame_id Target frame the footholds are expressed in
   * @return List of footholds representing the state at the given position
   */
  msgs::ErrorStatus getFootholds(msgs::FootholdArray& footholds, const Pose& feet_center, const std::string& frame_id);
  msgs::ErrorStatus getFootholds(FootholdArray& footholds, const Pose& feet_center, const std::string& frame_id);

  /**
   * @brief Generates footholds in neutral stance at given position
   * @param footholds [out] Generated footholds
   * @param feet_center Geometric feet center from which the footholds should be derived from.
   * The defined frame_id specifies the frame in which the footholds are expressed in.
   * @return List of footholds representing the state at the given position
   */
  msgs::ErrorStatus getFootholds(msgs::FootholdArray& footholds, const geometry_msgs::PoseStamped& feet_center);
  msgs::ErrorStatus getFootholds(FootholdArray& footholds, const geometry_msgs::PoseStamped& feet_center);

  /**
   * @brief Obtains best effort the footholds start configuration of the robot
   * @param footholds [out] Generated start footholds
   * @param frame_id Target frame the footholds are expressed in
   * @return List of footholds representing the start state
   */
  msgs::ErrorStatus getStartFootholds(msgs::FootholdArray& footholds, const std::string& frame_id);
  msgs::ErrorStatus getStartFootholds(FootholdArray& footholds, const std::string& frame_id);

private:
  msgs::ErrorStatus determineFootholds(msgs::FootholdArray& footholds, const geometry_msgs::PoseStamped& pose);

  msgs::ErrorStatus determineStartFootholds(msgs::FootholdArray& footholds, const std_msgs::Header& header);

  // service clients
  ros::ServiceClient generate_feet_pose_client_;
  ros::ServiceClient update_feet_client_; /// @todo provide functions such as moveToValid
};
}  // namespace l3_footstep_planning

#endif
