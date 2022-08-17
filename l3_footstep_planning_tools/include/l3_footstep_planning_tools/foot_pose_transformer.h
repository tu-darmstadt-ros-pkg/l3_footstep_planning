//=================================================================================================
// Copyright (c) 2013, Stefan Kohlbrecher, TU Darmstadt
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

#ifndef L3_FOOT_POSE_TRANSFORMER_H__
#define L3_FOOT_POSE_TRANSFORMER_H__

#include <ros/ros.h>

#include <l3_libs/types/types.h>
#include <l3_libs/singleton.h>
#include <l3_libs/robot_description/robot_description.h>

#include <l3_footstep_planning_libs/typedefs.h>
#include <l3_footstep_planning_libs/modeling/step_plan.h>

#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

namespace l3_footstep_planning
{
using namespace l3;
/**
 * Transforms (offsets) footholds between the planner's frame and the robot's frame.
 * The planner uses its (simplified) perspective of the footprint why all foothold
 * poses represent the center of the sole which is usually not part of the robot's kinematic.
 * In contrast, the robot usually uses for walking pre-defined frames/links as reference
 * coordinate system in which the foothold targets must be specified. The FootInfo
 * allows for specifing this offset between the (usually non-existend) sole frame and
 * the foot frame used by the robot's kinematic. Then this transformer will use this
 * specified offset to translate between both foothold representations.
 */
class FootPoseTransformer : public Singleton<FootPoseTransformer>
{
public:
  // typedefs
  typedef l3::SharedPtr<FootPoseTransformer> Ptr;
  typedef l3::SharedPtr<const FootPoseTransformer> ConstPtr;

  FootPoseTransformer();

  static bool initialize(ros::NodeHandle& nh);
  static bool initialize(RobotDescription::ConstPtr robot_description);

#if __cplusplus > 201402L
// auto return deduction does only work with c++14 and newer
  // specialized transforms
  template <typename T>
  static auto transformToPlannerFrame(T& p)
  {
    return transform(p, "planner");
  }

  template <typename T>
  static auto transformToRobotFrame(T& p)
  {
    return transform(p, "robot");
  }
#endif

protected:
  friend class FootPoseTransformerNode;

  static inline void transform(Pose& pose, const Transform& offset) { pose = pose * offset; }
  static void transform(geometry_msgs::Pose& pose, const Transform& offset);

  // interface for transformation of natives types
  static bool transform(Foothold& foothold, const std::string& target_frame);
  static bool transform(FootholdArray& feet, const std::string& target_frame);
  static bool transform(FootholdMap& feet, const std::string& target_frame);
  // Other types are not supported as they use const pointers which may point to hashed values.
  // This requires of deep copy the structures leading to a non expected behavior as all original pointers may be "lost".

  // interface for transformation of msg types
  static msgs::ErrorStatus transform(msgs::Foothold& foothold, const std::string& target_frame);
  static msgs::ErrorStatus transform(msgs::FootholdArray& feet, const std::string& target_frame);
  static msgs::ErrorStatus transform(msgs::FootStepData& foot_step, const std::string& target_frame);
  static msgs::ErrorStatus transform(msgs::FootStepDataArray& foot_steps, const std::string& target_frame);
  static msgs::ErrorStatus transform(msgs::Step& step, const std::string& target_frame);
  static msgs::ErrorStatus transform(msgs::StepArray& step_array, const std::string& target_frame);
  static msgs::ErrorStatus transform(msgs::StepQueue& step_queue, const std::string& target_frame);
  static msgs::ErrorStatus transform(msgs::StepPlan& step_plan, const std::string& target_frame);

  template <template <typename...> class Container>
  static msgs::ErrorStatus transform(Container<msgs::FootStepData>& container, const std::string& target_frame)
  {
    msgs::ErrorStatus status;
    for (typename Container<msgs::FootStepData>::iterator itr = container.begin(); itr != container.end(); itr++)
    {
      status += transform(itr->origin, target_frame);
      status += transform(itr->target, target_frame);
    }
    return status;
  }

#if __cplusplus <= 201402L
// Compatibility Code: Before c++14 we require to use type traits to resolve return type.
// However, this does only works when the members have been already declared, otherwise the
// statement would become ill-formed.
public:
  // specialized transforms
  template <typename T>
  static auto transformToPlannerFrame(T& p) -> decltype(FootPoseTransformer::transform(p, std::string()))
  {
    return transform(p, "planner");
  }

  template <typename T>
  static auto transformToRobotFrame(T& p) -> decltype(FootPoseTransformer::transform(p, std::string()))
  {
    return transform(p, "robot");
  }
protected:
#endif

  // Transformation: robot's "foot" tf frame -> planner foot frame (center of sole)
  std::map<FootIndex, Transform> offset_map_;
};

/**
 * Handler for convenient service calls
 */

msgs::ErrorStatus transform(msgs::Foothold& foot, ros::ServiceClient& transform_foot_pose_client, const std::string& target_frame);
msgs::ErrorStatus transform(msgs::FootholdArray& feet, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame);
msgs::ErrorStatus transform(msgs::FootStepData& foot_step, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame);
msgs::ErrorStatus transform(msgs::FootStepDataArray& foot_steps, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame);
msgs::ErrorStatus transform(msgs::Step& step, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame);
msgs::ErrorStatus transform(msgs::StepArray& step_array, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame);
msgs::ErrorStatus transform(msgs::StepQueue& step_queue, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame);
msgs::ErrorStatus transform(msgs::StepPlan& step_plan, ros::ServiceClient& transform_step_plan_client, const std::string& target_frame);

template <typename T>
msgs::ErrorStatus transformToPlannerFrame(T& p, ros::ServiceClient& client)
{
  return transform(p, client, "planner");
}

template <typename T>
msgs::ErrorStatus transformToRobotFrame(T& p, ros::ServiceClient& client)
{
  return transform(p, client, "robot");
}
}  // namespace l3_footstep_planning

#endif
