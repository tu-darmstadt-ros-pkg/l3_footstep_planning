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

#ifndef L3_FOOTSTEP_PLANNING_VIS_TOOLS_VISUALIZATION_H__
#define L3_FOOTSTEP_PLANNING_VIS_TOOLS_VISUALIZATION_H__

#include <ros/ros.h>
#include <tf/tf.h>

#include <visualization_msgs/MarkerArray.h>

#include <l3_libs/types/types.h>
#include <l3_libs/conversions/l3_msg_conversions.h>
#include <l3_libs/robot_description/robot_description.h>

#include <l3_vis/visualization.h>

#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

#include <l3_footstep_planning_libs/modeling/step_plan.h>

namespace l3_footstep_planning
{
inline visualization_msgs::MarkerArray stepPlanToFootMarkerArray(const msgs::StepPlan& step_plan, const RobotDescription& robot_description, bool add_step_index = true, const std::string& ns = "step_plan")
{
  return stepPlanToFootMarkerArray(step_plan.plan.steps, robot_description, add_step_index);
}
inline visualization_msgs::MarkerArray stepPlanToFootMarkerArray(const StepPlan& step_plan, const RobotDescription& robot_description, bool add_step_index = true, const std::string& ns = "step_plan")
{
  msgs::StepArray array;
  stepArrayL3ToMsg(step_plan.getSteps().asArray(), array);
  return stepPlanToFootMarkerArray(array, robot_description, add_step_index, ns);
}

inline visualization_msgs::MarkerArray stepPlanToUpperBodyMarkerArray(const msgs::StepPlan& step_plan, const RobotDescription& robot_description, bool add_step_index = true, const std::string& ns = "upper_body")
{
  return stepPlanToUpperBodyMarkerArray(step_plan.start, step_plan.plan.steps, robot_description, add_step_index);
}
inline visualization_msgs::MarkerArray stepPlanToUpperBodyMarkerArray(const StepPlan& step_plan, const RobotDescription& robot_description, bool add_step_index = true, const std::string& ns = "upper_body")
{
  msgs::FootholdArray start;
  footholdArrayL3ToMsg(step_plan.getStart(), start);

  msgs::StepArray array;
  stepArrayL3ToMsg(step_plan.getSteps().asArray(), array);
  return stepPlanToUpperBodyMarkerArray(start, array, robot_description, add_step_index, ns);
}
}  // namespace l3_footstep_planning

#endif
