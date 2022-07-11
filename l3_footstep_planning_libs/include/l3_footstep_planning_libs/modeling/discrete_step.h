//=================================================================================================
// Copyright (c) 2022, alexander stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_LIBS_DISCRETE_STEP_H__
#define L3_FOOTSTEP_PLANNING_LIBS_DISCRETE_STEP_H__

#include <l3_libs/robot_description/base_info.h>

#include <l3_math/angles.h>

#include <l3_footstep_planning_libs/helper.h>
#include <l3_footstep_planning_libs/modeling/planning_state.h>

namespace l3_footstep_planning
{
/**
 * @brief DiscreteStep represents the discrete translation. The constructor takes the
 * parameters relative to the base center, but afterwards a DiscreteStep represents
 * the possible placement relative to the robot body (center). This means that the
 * final Footstep already considers the neutral stance pose of the base relative to
 * the robot's body.
 */
template <class Type>
class DiscreteStep
{
  /// Typedef representing the (discretized) translation of the step.
  typedef std::pair<double, double> DiscreteStepXY;

public:
  // typedefs
  typedef l3::SharedPtr<DiscreteStep> Ptr;
  typedef l3::SharedPtr<const DiscreteStep> ConstPtr;

  /**
   * @brief The constructor takes the continuous translation and rotation
   * of the step and calculates the respective discretized step
   * based on the parameters of the discretization.
   * @param neutral_stance Neutral stance pose of the base relative to robot center.
   * @param link_idx Index to the link that this step does referes to
   * @param dx The (continuous) translation in x direction given relative to the base center.
   * @param dy The (continuous) translation in y direction given relative to the base center.
   * @param droll The (continuous) roll rotation.
   * @param dpitch The (continuous) pitch rotation.
   * @param dyaw The (continuous) yaw rotation.
   * @param res Planning resolution
   */
  DiscreteStep(const Pose& neutral_stance, const LinkIndex& link_idx, double dx, double dy, double droll, double dpitch, double dyaw, double step_cost,
               const DiscreteResolution& res)
    : res_(res)
    , link_idx_(link_idx)
    , droll_(neutral_stance.pitch() + droll)
    , dpitch_(neutral_stance.pitch() + dpitch)
    , dyaw_(neutral_stance.yaw() + dyaw)
    , step_cost_(step_cost)
    , backward_steps_(res_.numAngleBins())
    , forward_steps_(res_.numAngleBins())
  {
    double step_x;
    double step_y;

    for (int global_theta = 0; global_theta < res_.numAngleBins(); global_theta++)
    {
      calculateBackwardStep(neutral_stance, res_.toContAngle(global_theta), dx, dy, step_x, step_y);
      backward_steps_[global_theta] = DiscreteStepXY(step_x, step_y);

      calculateForwardStep(neutral_stance, res_.toContAngle(global_theta), dx, dy, step_x, step_y);
      forward_steps_[global_theta] = DiscreteStepXY(step_x, step_y);
    }
  }

  /**
   * @brief Reverse this step on a given planning state.
   * @param current The current robot state
   * @return The reversed planning state, i.e. the state the robot was in
   * if this step had not been performed.
   */
  l3::SharedPtr<Type> getPredecessor(const Pose& robot_pose) const
  {
    int yaw = res_.toDiscAngle(robot_pose.yaw());

    // theta has to be in [0..ivNumAngleBins)
    while (yaw < 0)
      yaw += res_.numAngleBins();
    while (yaw >= res_.numAngleBins())
      yaw -= res_.numAngleBins();

    const DiscreteStepXY& xy = backward_steps_[yaw];

    return l3::makeShared<Type>(link_idx_, robot_pose.x() + xy.first, robot_pose.y() + xy.second, robot_pose.z(), 0.0, 0.0, l3::normalizeAngle(robot_pose.yaw() - dyaw_));
  }
  l3::SharedPtr<Type> getPredecessor(const State& current) const
  {
    if constexpr (std::is_same<Type, Foothold>::value)
    {
      ROS_ASSERT(current.getFoothold(foot_idx_));
      return getPredecessor(current.getFeetCenter());
    }
    else if constexpr (std::is_same<Type, FloatingBase>::value)
    {
      ROS_ASSERT(current.hasFloatingBases());
      return getPredecessor(current.getFloatingBase(BaseInfo::MAIN_BODY_IDX)->pose());
    }
    else
      return l3::SharedPtr<Type>();
  }
  inline l3::SharedPtr<Type> getPredecessor(State::ConstPtr current) const { return getPredecessor(*current); }

  /**
   * @brief Performs this step (translation and rotation) on a given
   * planning state.
   * @param current The current robot state
   * @return The resulting planning state.
   */
  l3::SharedPtr<Type> getSuccessor(const Pose& robot_pose) const
  {
    int yaw = res_.toDiscAngle(robot_pose.yaw());

    // theta has to be in [0..ivNumAngleBins)
    while (yaw < 0)
      yaw += res_.numAngleBins();
    while (yaw >= res_.numAngleBins())
      yaw -= res_.numAngleBins();

    const DiscreteStepXY& xy = forward_steps_[yaw];

    return l3::makeShared<Type>(link_idx_, robot_pose.x() + xy.first, robot_pose.y() + xy.second, robot_pose.z(), 0.0, 0.0, l3::normalizeAngle(robot_pose.yaw() + dyaw_));
  }
  l3::SharedPtr<Type> getSuccessor(const State& current) const
  {
    if constexpr (std::is_same<Type, Foothold>::value)
    {
      ROS_ASSERT(current.getFoothold(foot_idx_));
      return getSuccessor(current.getFeetCenter());
    }
    else if constexpr (std::is_same<Type, FloatingBase>::value)
    {
      ROS_ASSERT(current.hasFloatingBases());
      return getSuccessor(current.getFloatingBase(BaseInfo::MAIN_BODY_IDX)->pose());
    }
    else
      return l3::SharedPtr<Type>();
  }
  inline SharedPtr<Type> getSuccessor(State::ConstPtr current) const { return getSuccessor(*current); }

  const LinkIndex& getLinkIndex() const { return link_idx_; }
  double getStepCost() const { return step_cost_; }

protected:
  /**
   * @brief Discretizes the translation of the footstep for a certain
   * orientation of a possible state.
   * @param neutral_stance Neutral stance pose of leg relative to robot center
   * @param global_theta The orientation of the possible state.
   * @param dx The translation in x direction.
   * @param dy The translation in y direction.
   * @param footstep_x The resulting translation in x direction.
   * @param footstep_y The resulting translation in y direction.
   */
  void calculateBackwardStep(const Pose& neutral_stance, double global_theta, double dx, double dy, double& step_x, double& step_y) const
  {
    calculateStep(global_theta, neutral_stance.x() - dx, neutral_stance.y() - dy, step_x, step_y);
  }
  void calculateForwardStep(const Pose& neutral_stance, double global_theta, double dx, double dy, double& step_x, double& step_y) const
  {
    calculateStep(global_theta, neutral_stance.x() + dx, neutral_stance.y() + dy, step_x, step_y);
  }
  void calculateStep(double global_theta, double dx, double dy, double& step_x, double& step_y) const
  {
    double theta_cos = cos(global_theta);
    double theta_sin = sin(global_theta);

    step_x = theta_cos * dx - theta_sin * dy;
    step_y = theta_sin * dx + theta_cos * dy;
  }

  /// The parameter for the discretization
  DiscreteResolution res_;

  /// Base Index of represented base
  LinkIndex link_idx_;

  /// The rotation of the step.
  double droll_;
  double dpitch_;
  double dyaw_;

  std::vector<DiscreteStepXY> backward_steps_;
  std::vector<DiscreteStepXY> forward_steps_;

  double step_cost_;
};
}  // namespace l3_footstep_planning

#endif
