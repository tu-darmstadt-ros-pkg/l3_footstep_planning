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

#ifndef L3_FOOTSTEP_PLANNING_PLANNING_FEEDBACK_HANDLER_H__
#define L3_FOOTSTEP_PLANNING_PLANNING_FEEDBACK_HANDLER_H__

#include <boost/function.hpp>

#include <l3_footstep_planning_libs/modeling/planning_state.h>

#include <l3_footstep_planner/environment_parameters.h>

namespace l3_footstep_planning
{
/**
 * @brief A class defining a footstep planner environment for humanoid
 * robots used by the SBPL to perform planning tasks.
 *
 * The environment keeps track of all the planning states expanded during
 * the search. Each planning state can be accessed via its ID. Furthermore
 */
class PlanningFeedbackHandler
{
public:
  // typedefs
  typedef l3::SharedPtr<PlanningFeedbackHandler> Ptr;
  typedef l3::SharedPtr<const PlanningFeedbackHandler> ConstPtr;

  PlanningFeedbackHandler(ros::NodeHandle& nh);

  void setParameters(const EnvironmentParameters& params);
  void setFrameId(const std::string& frame_id);
  void setFeedbackCB(boost::function<void(msgs::PlanningFeedback)>& feedback_cb);

  void reset(); // clears data and stops callback
  void clear(); // clears data

  void start(double rate = 1.0);
  void stop();

  void stateExpanded(PlanningState::ConstPtr state);
  void stateVisited(PlanningState::ConstPtr state);

protected:
  void publishFeedback(const ros::TimerEvent& event);

  mutable Mutex timer_mutex_;
  mutable Mutex data_mutex_;

  // feedback callback
  ros::Timer feedback_timer_;
  boost::function<void(msgs::PlanningFeedback)> feedback_cb_;

  // other parameters required for feedback
  std::string frame_id_;
  bool forward_search_;

  // containers for collecting feedback data
  StepConstPtrArray visited_steps_;
  PlanningState::ConstPtr last_visited_state_;
};
}  // namespace l3_footstep_planning

#endif
