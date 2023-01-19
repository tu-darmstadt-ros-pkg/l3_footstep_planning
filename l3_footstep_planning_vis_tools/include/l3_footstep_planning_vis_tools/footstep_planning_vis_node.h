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

#ifndef L3_FOOTSTEP_PLANNING_VIS_NODE_H__
#define L3_FOOTSTEP_PLANNING_VIS_NODE_H__

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include <vigir_pluginlib/plugin_aggregator.h>

#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

#include <l3_footstep_planning_vis_tools/base/planning_vis_plugin.h>

namespace l3_footstep_planning
{
class FootstepPlanningVisNode
{
public:
  FootstepPlanningVisNode(ros::NodeHandle& nh);

protected:
  bool loadPluginSet(const std::string& name);

  inline void loadPluginSetCallBack(const std_msgs::StringConstPtr& name) { loadPluginSet(name->data); }
  void stepPlanRequestVisCallback(const msgs::StepPlanRequestConstPtr& step_plan_request);
  void stepPlanVisCallback(const msgs::StepPlanConstPtr& step_plan);
  void planningFeedbackCallback(const msgs::PlanningFeedbackConstPtr& planning_feedback);

  void clearVisualization();

  void updateMarker(const visualization_msgs::MarkerArray& markers);
  void publishMarkerArray();

  vigir_pluginlib::PluginAggregator<PlanningVisPlugin> vis_plugins_;

  std::map<std::string, visualization_msgs::MarkerArray> markers_;

  // timer
  ros::Timer auto_publisher_;

  // subscriber
  ros::Subscriber load_plugin_set_sub_;
  ros::Subscriber step_plan_request_vis_sub_;
  ros::Subscriber step_plan_vis_sub_;
  ros::Subscriber planning_feedback_sub_;

  // publisher
  ros::Publisher marker_pub_;
};
}  // namespace l3_footstep_planning

#endif
