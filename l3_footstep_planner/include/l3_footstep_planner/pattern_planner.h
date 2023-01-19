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

#ifndef L3_PATTERN_PLANNER_H__
#define L3_PATTERN_PLANNER_H__

#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

#include <l3_footstep_planning_libs/modeling/planning_state.h>
#include <l3_footstep_planning_libs/modeling/step_plan.h>
#include <l3_footstep_planning_libs/modeling/foot_step_action.h>

namespace l3_footstep_planning
{
class PatternPlanner
{
public:
  PatternPlanner(const DiscreteResolution& res = DiscreteResolution(0.0001, 0.0001, 0.0001, 3600));

  void reset();

  msgs::ErrorStatus planPattern(StepPtrArray& path, StateHashed::ConstPtr start_state, const l3_footstep_planning_msgs::StepPlanRequestService::Request& req);

protected:
  /**
   * @brief Dispatches the pattern planning request and generates suitable footsteps.
   * @param params Parameters for pattern planning request
   * @return Status message (empty when no issues occured)
   */
  msgs::ErrorStatus configureGenerator(const msgs::PatternParameters& params);

  /**
   * @brief Calls the pattern generator based on the given walking request.
   * @param params Parameters for pattern planning request
   * @param start_foot_idx Index of first leg to move. If set to AUTO_START_FOOT_IDX the generator will automatically select one.
   * @return Status message (empty when no issues occured)
   */
  msgs::ErrorStatus generateWalkPattern(const msgs::PatternParameters& params, const FootIndex& start_foot_idx);

  /**
   * @brief Creates sequence to bring the robot into the neutral stance.
   * @param params Parameters for pattern planning request
   * @param force_all If set to true all legs will be moved despite if they are already in neutral stance pose
   * @return Status message (empty when no issues occured)
   */
  msgs::ErrorStatus generateNeutralStancePattern(const msgs::PatternParameters& params, bool force_all = false);

  /**
   * @brief Extracts current plan from the generator
   * @param path [out] Resulting footstep plan
   * @return Status message (empty when no issues occured)
   */
  msgs::ErrorStatus extractPath(StepPtrArray& path);

  /**
   * @brief Generates footsteps for each leg based on given delta relative to the neutral stance.
   * @param dx Delta in x
   * @param dy Delta in y
   * @param dyaw Delta in yaw
   * @return Map of footsteps for each leg
   */
  std::map<FootIndex, FootStepAction> generateFootsteps(double dx, double dy, double dyaw) const;

  /// @brief Returns the foot pose of a leg for a given robot pose.
  Foothold getFootPose(const Pose& robot, const FootIndex& foot_idx, double dx = 0.0, double dy = 0.0, double dyaw = 0.0) const;

  DiscreteResolution res_;

  std::map<FootIndex, FootStepAction> footsteps_;
  bool single_step_mode_;
  bool change_z_;

  UID fake_uid_;  // required to omit StateSpaceManager (that would discretize the foothold)
  PlanningState::Ptr pstate_;
  std::vector<PlanningState::Ptr> plan_;
};
}  // namespace l3_footstep_planning

#endif
