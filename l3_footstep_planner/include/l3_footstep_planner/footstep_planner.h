//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNER_H__
#define L3_FOOTSTEP_PLANNER_H__

#include <ros/ros.h>
#include <tf/tf.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#include <vigir_generic_params/generic_params_msgs.h>
#include <vigir_generic_params/parameter_manager.h>

#include <vigir_pluginlib/plugin_manager.h>

#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

#include <l3_footstep_planning_libs/math.h>
#include <l3_footstep_planning_libs/modeling/state.h>

#include <l3_footstep_planner/environment_parameters.h>
#include <l3_footstep_planner/footstep_planner_environment.h>
#include <l3_footstep_planner/planning_feedback_handler.h>

namespace l3_footstep_planning
{
/**
 * @brief A class to control the interaction between ROS and the footstep
 * planner.
 */
class FootstepPlanner
{
public:
  typedef boost::function<void(msgs::StepPlanRequestService::Response)> ResultCB;
  typedef boost::function<void(msgs::PlanningFeedback)> FeedbackCB;
  typedef boost::function<void()> PreemptCB;

  FootstepPlanner(ros::NodeHandle& nh);
  virtual ~FootstepPlanner();

  bool isPlanning() const;

  bool setParams(const vigir_generic_params::ParameterSet& params);

  msgs::ErrorStatus updateFoot(msgs::Foothold& foot, uint8_t mode, bool transform = true) const;
  msgs::ErrorStatus updateFeet(msgs::FootholdArray& footholds, uint8_t mode, bool transform = true) const;
  msgs::ErrorStatus updateStepPlan(msgs::StepPlan& result, uint8_t mode, const std::string& param_set_name = std::string(), bool transform = true) const;

  msgs::ErrorStatus stepPlanRequest(const l3_footstep_planning_msgs::StepPlanRequestService::Request& req, ResultCB result_cb = ResultCB(), FeedbackCB feedback_cb = FeedbackCB(), PreemptCB preempt_cb = PreemptCB());
  /// @brief Service handle to plan footsteps.
  bool stepPlanRequestService(msgs::StepPlanRequestService::Request& req, msgs::StepPlanRequestService::Response& resp);

  /// @brief stops thread running planning
  void preemptPlanning();

  // typedefs
  typedef l3::SharedPtr<FootstepPlanner> Ptr;
  typedef l3::SharedPtr<const FootstepPlanner> ConstPtr;

protected:
  /**
   * @brief Start a planning task from scratch (will delete information
   * of previous planning tasks). Map and start, goal poses need to be
   * set beforehand.
   *
   * @return Success of planning.
   */
  msgs::ErrorStatus planSteps(msgs::StepPlanRequestService::Request& req);

  /// @brief plans stepping
  msgs::ErrorStatus planPattern(msgs::StepPlanRequestService::Request& req);

  /// @brief extracts step plan response from planning result
  bool finalizeStepPlan(msgs::StepPlanRequestService::Request& req, msgs::StepPlanRequestService::Response& resp, bool post_process);

  /// @brief: starts planning in a new thread
  void startPlanning(msgs::StepPlanRequestService::Request& req);
  /// @brief: method used in seperate thread
  void doPlanning(msgs::StepPlanRequestService::Request& req);

  bool findNearestValidFoothold(Foothold& s) const;

  /**
   * @brief Sets the start pose as position of left and right footsteps.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setStart(const State& state, bool ignore_collision = false);

  /**
   * @brief Sets the start pose
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setStart(const msgs::StepPlanRequest& req, bool ignore_collision = false);

  /**
   * @brief Sets the goal pose as position of left and right footsteps.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setGoal(const State& state, bool ignore_collision = false);

  /**
   * @brief Sets the goal pose
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setGoal(const msgs::StepPlanRequest& req, bool ignore_collision = false);

  /// @return Costs of the planned footstep path.
  inline double getPathCosts() const { return path_cost_; }

  /// @return Number of expanded states.
  inline size_t getNumExpandedStates() const { return planner_->get_n_expands(); }

  /// @return Number of planned foot poses.
  inline size_t getNumFootPoses() const { return path_.size(); }

  /// @return Size of the planned path.
  inline int getPathSize() { return path_.size(); }

  inline State::ConstPtr getStartState() { return start_state_; }

  /// @brief Reset the previous planning information.
  void reset();

  /// @brief Reset and reinitialize the environment.
  void resetTotally();

  /// @return True if for the current start and goal pose a path exists.
  inline bool pathExists() { return !path_.empty(); }

  /**
   * @brief Extracts the path (list of foot poses) from a list of state
   * IDs calculated by the SBPL.
   */
  bool extractPath(const std::vector<int>& state_ids, const UID& start_uid, const UID& goal_uid);

  /**
   * @brief Starts the planning task in the underlying SBPL.
   *
   * NOTE: Never call this directly. Always use either plan() or replan() to
   * invoke this method.
   */
  bool plan(ReplanParams& params);

  /// @brief Sets the planning algorithm used by SBPL.
  void setPlanner();

  // threading
  mutable boost::recursive_mutex planner_mutex_;
  boost::thread planning_thread_;

  ResultCB result_cb_;
  PreemptCB preempt_cb_;

  // SBPL interface
  FootstepPlannerEnvironment::Ptr planner_environment_;
  l3::SharedPtr<SBPLPlanner> planner_;

  StepPtrArray path_;
  double path_cost_;

  StateHashed::ConstPtr start_state_;
  StateHashed::ConstPtr goal_state_;

  // handlers
  PlanningFeedbackHandler feedback_handler_;

  // parameters
  std::string frame_id_;
  EnvironmentParameters::Ptr env_params_;
  FootIndex start_foot_idx_;
  bool start_pose_set_up_, goal_pose_set_up_;

  bool print_use_masks_;
  bool print_solution_;

  // counter to be used as sequence number
  unsigned int step_plan_uid_;
};
}  // namespace l3_footstep_planning

#endif
