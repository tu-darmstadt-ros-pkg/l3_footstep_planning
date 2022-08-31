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

#ifndef L3_FOOTSTEP_PLANNING_MSGS_H__
#define L3_FOOTSTEP_PLANNING_MSGS_H__

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>

// messages
#include <l3_footstep_planning_msgs/EditStep.h>
#include <l3_footstep_planning_msgs/ErrorStatus.h>
#include <l3_footstep_planning_msgs/FeetPoseRequest.h>
#include <l3_footstep_planning_msgs/PatternParameters.h>
#include <l3_footstep_planning_msgs/PatternGeneratorParameters.h>
#include <l3_footstep_planning_msgs/PlanningFeedback.h>
#include <l3_footstep_planning_msgs/StepPlanFeedback.h>
#include <l3_footstep_planning_msgs/StepPlanRequest.h>
#include <l3_footstep_planning_msgs/StepPlan.h>
#include <l3_footstep_planning_msgs/UpdateMode.h>

// services
#include <l3_footstep_planning_msgs/EditStepService.h>
#include <l3_footstep_planning_msgs/GenerateFeetPoseService.h>
#include <l3_footstep_planning_msgs/GetStepPlanService.h>
#include <l3_footstep_planning_msgs/PatternGeneratorParametersService.h>
#include <l3_footstep_planning_msgs/SetStepPlanService.h>
#include <l3_footstep_planning_msgs/StepPlanRequestService.h>
#include <l3_footstep_planning_msgs/StitchStepPlanService.h>
#include <l3_footstep_planning_msgs/StitchStepPlanService.h>
#include <l3_footstep_planning_msgs/TransformFeetPosesService.h>
#include <l3_footstep_planning_msgs/TransformFootPoseService.h>
#include <l3_footstep_planning_msgs/TransformStepPlanService.h>
#include <l3_footstep_planning_msgs/StitchStepPlanService.h>
#include <l3_footstep_planning_msgs/UpdateFeetService.h>
#include <l3_footstep_planning_msgs/UpdateFootService.h>
#include <l3_footstep_planning_msgs/UpdateStepPlanService.h>

// actions
#include <l3_footstep_planning_msgs/EditStepAction.h>
#include <l3_footstep_planning_msgs/ExecuteStepPlanAction.h>
#include <l3_footstep_planning_msgs/GenerateFeetPoseAction.h>
#include <l3_footstep_planning_msgs/GetStepPlanAction.h>
#include <l3_footstep_planning_msgs/SetStepPlanAction.h>
#include <l3_footstep_planning_msgs/StepPlanRequestAction.h>
#include <l3_footstep_planning_msgs/StitchStepPlanAction.h>
#include <l3_footstep_planning_msgs/UpdateFeetAction.h>
#include <l3_footstep_planning_msgs/UpdateFootAction.h>
#include <l3_footstep_planning_msgs/UpdateStepPlanAction.h>

namespace l3_footstep_planning
{
namespace msgs
{
using namespace l3_msgs;
using namespace l3_footstep_planning_msgs;
}  // namespace msgs

// Extension to ErrorStatus message
msgs::ErrorStatus operator+(const msgs::ErrorStatus& lhs, const msgs::ErrorStatus& rhs);
msgs::ErrorStatus operator+=(msgs::ErrorStatus& lhs, const msgs::ErrorStatus& rhs);

msgs::ErrorStatus isConsistent(const msgs::StepPlan& result);

std::string ErrorStatusCodeToString(unsigned int error);
std::string WarningStatusCodeToString(unsigned int warning);

msgs::ErrorStatus createErrorStatus(const std::string& context, unsigned int error, const std::string& error_msg, unsigned int warning, const std::string& warning_msg,
                                    bool output = true, double throttle_rate = 0.0);
msgs::ErrorStatus ErrorStatusError(unsigned int error, const std::string& context, const std::string& error_msg, bool output = true, double throttle_rate = 0.0);
msgs::ErrorStatus ErrorStatusWarning(unsigned int warning, const std::string& context, const std::string& warning_msg, bool output = true, double throttle_rate = 0.0);

bool hasError(const msgs::ErrorStatus& status);
bool hasError(const msgs::ErrorStatus& status, unsigned int code);
bool hasWarning(const msgs::ErrorStatus& status);
bool hasWarning(const msgs::ErrorStatus& status, unsigned int code);
bool isOk(const msgs::ErrorStatus& status);

std::string toString(const msgs::ErrorStatus& error_status);

template <typename Tin, typename Tout>
void copyPosition(const Tin& p_in, Tout& p_out)
{
  p_out.x = p_in.x;
  p_out.y = p_in.y;
  p_out.z = p_in.z;
}
}  // namespace l3_footstep_planning

#endif
