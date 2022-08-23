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

#ifndef FOOTSTEP_PLANNER_NODE_H__
#define FOOTSTEP_PLANNER_NODE_H__

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

#include <l3_footstep_planner/footstep_planner.h>

namespace l3_footstep_planning
{
/**
 * @brief Wrapper class for FootstepPlanner, providing callbacks for
 * the node functionality.
 */
class FootstepPlannerNode
{
public:
  FootstepPlannerNode(ros::NodeHandle& nh);

  virtual void initPlugins(ros::NodeHandle& nh);
  virtual void initialize(ros::NodeHandle& nh);

protected:
  // callbacks
  void planningResultCallback(const msgs::StepPlanRequestService::Response& resp);
  void planningResultActionCallback(const msgs::StepPlanRequestService::Response& resp, SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as);

  void planningFeedbackCallback(const msgs::PlanningFeedback& feedback);
  void planningFeedbackActionCallback(const msgs::PlanningFeedback& feedback, SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as);

  void planningPreemptionActionCallback(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as);

  // subscriber
  void setParams(const std_msgs::StringConstPtr& params_name);
  void stepPlanRequest(const msgs::StepPlanRequestConstPtr& plan_request);
  void goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose);

  // service calls
  bool stepPlanRequestService(msgs::StepPlanRequestService::Request& req, msgs::StepPlanRequestService::Response& resp);
  bool updateFootService(msgs::UpdateFootService::Request& req, msgs::UpdateFootService::Response& resp);
  bool updateFeetService(msgs::UpdateFeetService::Request& req, msgs::UpdateFeetService::Response& resp);
  bool updateStepPlanService(msgs::UpdateStepPlanService::Request& req, msgs::UpdateStepPlanService::Response& resp);

  // action server calls
  void stepPlanRequestAction(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as);
  void stepPlanRequestPreempt(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as);
  void updateFootAction(SimpleActionServer<msgs::UpdateFootAction>::Ptr& as);
  void updateFeetAction(SimpleActionServer<msgs::UpdateFeetAction>::Ptr& as);
  void updateStepPlanAction(SimpleActionServer<msgs::UpdateStepPlanAction>::Ptr& as);

  // subscribers
  ros::Subscriber set_active_parameter_set_sub_;
  ros::Subscriber step_plan_request_sub_;
  ros::Subscriber goal_pose_sub_;

  // publisher
  ros::Publisher step_plan_pub_;
  ros::Publisher step_plan_request_vis_pub_;
  ros::Publisher step_plan_vis_pub_;
  ros::Publisher error_status_pub_;
  ros::Publisher temp_step_plan_pub_;
  ros::Publisher feedback_pub_;

  // service clients
  ros::ServiceClient generate_feet_pose_client_;

  // service servers
  ros::ServiceServer step_plan_request_srv_;
  ros::ServiceServer update_foot_srv_;
  ros::ServiceServer update_feet_srv_;
  ros::ServiceServer update_step_plan_srv_;

  // action servers
  SimpleActionServer<msgs::StepPlanRequestAction>::Ptr step_plan_request_as_;
  SimpleActionServer<msgs::UpdateFootAction>::Ptr update_foot_as_;
  SimpleActionServer<msgs::UpdateFeetAction>::Ptr update_feet_as_;
  SimpleActionServer<msgs::UpdateStepPlanAction>::Ptr update_step_plan_as_;

  mutable boost::recursive_mutex step_plan_request_as_mutex_;

  FootstepPlanner::Ptr footstep_planner_;
};
}  // namespace l3_footstep_planning
#endif
