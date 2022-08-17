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

#ifndef L3_FOOTSTEP_PLANNING_MSGS_TEST_HELPER_H__
#define L3_FOOTSTEP_PLANNING_MSGS_TEST_HELPER_H__

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <l3_libs/types/types.h>
#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

#include <l3_footstep_planning_libs/modeling/step_plan.h>

namespace l3_footstep_planning
{
void isEqualTest(const std_msgs::String& exp, const std_msgs::String& res);
void isEqualTest(const std_msgs::Header& exp, const std_msgs::Header& res);

void isEqualTest(const geometry_msgs::Point& exp, const geometry_msgs::Point& res, double abs_error = 0.0);
void isEqualTest(const geometry_msgs::Quaternion& exp, const geometry_msgs::Quaternion& res, double abs_error = 0.0);
void isEqualTest(const geometry_msgs::Pose& exp, const geometry_msgs::Pose& res, double abs_error = 0.0);

void isEqualTest(const msgs::Foothold& exp, const msgs::Foothold& res, double abs_error = 0.0);
void isEqualTest(const msgs::FootholdArray& exp, const msgs::FootholdArray& res, double abs_error = 0.0);
void isEqualTest(const msgs::FloatingBase& exp, const msgs::FloatingBase& res, double abs_error = 0.0);
void isEqualTest(const msgs::FootStepData& exp, const msgs::FootStepData& res, double abs_error = 0.0);
void isEqualTest(const msgs::Step& exp, const msgs::Step& res, double abs_error = 0.0);
void isEqualTest(const msgs::StepPlan& exp, const msgs::StepPlan& res, double abs_error = 0.0);

void isEqualTest(const StepPlan& exp, const StepPlan& res, double abs_error = 0.0);
}  // namespace l3_footstep_planning

#endif
