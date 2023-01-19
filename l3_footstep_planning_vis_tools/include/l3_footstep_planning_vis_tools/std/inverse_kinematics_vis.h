//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
// Based on http://wiki.ros.org/footstep_planner by Johannes Garimort and Armin Hornung
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

#ifndef L3_INVERSE_KINEMATICS_VIS_H__
#define L3_INVERSE_KINEMATICS_VIS_H__

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>

#include <robot_state_publisher/robot_state_publisher.h>

#include <l3_plugins/base/kinematics_plugin.h>

#include <l3_footstep_planning_vis_tools/base/planning_vis_plugin.h>
#include <l3_footstep_planning_vis_tools/visualization.h>

namespace l3_footstep_planning
{
/**
 * Following naming conventions are used in this plugin:
 * root -> root element of the entire robot chain (usually base_link)
 * base -> last common link in chain before legs are branching off (truncated chain) from which usually the IK is computed (e.g. pelvis_link for bipeds)
 * center -> ground projected pose of center of convex hull when all feet are in neutral stance
 */
class InverseKinematicsVis : public PlanningVisPlugin
{
public:
  InverseKinematicsVis();

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

  void clear() override;

  void visualize(const msgs::StepPlanRequest& step_plan_request) override;
  void visualize(const msgs::PlanningFeedback& planning_feedback) override;

  void visualize(const StepPlan& step_plan) override;

protected:
  virtual void visualize(const Step& step, const std::string& frame_id);

  /**
   * @brief Generates IK-based visualization corresponding to the given foothold locations and floating base.
   * @param footholds target footholds for IK to solve
   * @param floating_base target floating base for IK to solve
   * @param frame_id frame id in which the footholds are given;
   * Note: A tf transform from this frame to the used planning_vis tf tree will be published.
   */
  virtual void visualize(const FootholdArray& footholds, const FloatingBaseMap& floating_bases, const std::string& frame_id);

  virtual bool calcLegIK(const Pose& base_pose, const Foothold& foothold, std::vector<double>& q) const;

  bool calcNeutralStanceIK(std::map<LegIndex, std::vector<double>>& leg_joint_states) const;

  void jointStateCB(const sensor_msgs::JointStateConstPtr& msg);
  void rvizVisualToolsGuiCB(const sensor_msgs::JoyConstPtr& msg);

  void publishTF(const ros::TimerEvent& event);
  void animate(const ros::TimerEvent& event);

  void scheduleAnimationStep(double time_step);

  std::string tf_prefix_;

  std::map<LegIndex, std::vector<double>> def_leg_joint_states_;  // default joint states of legs used as inital solution for IK
  std::map<std::string, double> cur_joint_states_;                // real joint states of robot
  std::map<std::string, double> vis_joint_states_;                // ik solution for visualization

  Transform base_to_root_;
  geometry_msgs::TransformStamped world_to_root_;

  StepPlan current_step_plan_;
  StepIndex current_step_idx_;

  // params
  double animate_time_scale_;

  bool visualize_feedback_;

  // subscriber
  ros::Subscriber joint_state_sub_;
  ros::Subscriber rviz_visual_tools_gui_sub_;

  // publisher
  tf::TransformBroadcaster tf_broadcaster_;
  l3::SharedPtr<robot_state_publisher::RobotStatePublisher> robot_state_publisher_;

  // timer
  ros::Timer publish_tf_timer_;
  ros::Timer animation_timer_;
};
}  // namespace l3_footstep_planning

#endif
