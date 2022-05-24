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

#ifndef L3_FOOTSTEP_PLANNING_LIBS_BASESTEP_H__
#define L3_FOOTSTEP_PLANNING_LIBS_BASESTEP_H__

#include <l3_footstep_planning_libs/modeling/planning_state.h>

namespace l3_footstep_planning
{
/**
 * @brief Basestep represents the discrete translation. The constructor takes the
 * parameters relative to the base center, but afterwards a Basestep represents
 * the possible placement relative to the robot body (center). This means that the
 * final Footstep already considers the neutral stance pose of the base relative to
 * the robot's body.
 */
class Basestep
{
  /// Typedef representing the (discretized) translation of the basestep.
  typedef std::pair<double, double> BasestepXY;

public:
  // typedefs
  typedef l3::SharedPtr<Basestep> Ptr;
  typedef l3::SharedPtr<const Basestep> ConstPtr;

  /**
   * @brief The constructor takes the continuous translation and rotation
   * of the basestep and calculates the respective discretized basestep
   * based on the parameters of the discretization.
   * @param neutral_stance Neutral stance pose of the base relative to robot center.
   * @param dx The (continuous) translation in x direction given relative to the base center.
   * @param dy The (continuous) translation in y direction given relative to the base center.
   * @param dyaw The (continuous) rotation.
   * @param res Planning resolution
   */
  Basestep(const Pose& neutral_stance, const BaseIndex& foot_idx, double dx, double dy, double dyaw, double dpitch, double step_cost, const DiscreteResolution& res);

  /**
   * @brief Reverse this basestep on a given planning state.
   * @param current The current robot state
   * @return The reversed planning state, i.e. the state the robot was in
   * if this basestep had not been performed.
   */
  FloatingBase::Ptr getPredFloatingBase(const Pose& robot_pose) const;
  FloatingBase::Ptr getPredFloatingBase(const State& current) const;
  inline FloatingBase::Ptr getPredFloatingBase(State::ConstPtr current) const { return getPredFloatingBase(*current); }

  /**
   * @brief Performs this basestep (translation and rotation) on a given
   * planning state.
   * @param current The current robot state
   * @return The resulting planning state.
   */
  FloatingBase::Ptr getSuccFloatingBase(const Pose& robot_pose) const;
  FloatingBase::Ptr getSuccFloatingBase(const State& current) const;
  inline FloatingBase::Ptr getSuccFloatingBase(State::ConstPtr current) const { return getSuccFloatingBase(*current); }

  const BaseIndex& getBaseIndex() const { return base_idx_; }
  double getStepCost() const { return step_cost_; }

private:
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
  void calculateBackwardStep(const Pose& neutral_stance, double global_theta, double dx, double dy, double& basestep_x, double& basestep_y) const;
  void calculateForwardStep(const Pose& neutral_stance, double global_theta, double dx, double dy, double& basestep_x, double& basestep_y) const;
  void calculateStep(double global_theta, double dx, double dy, double& basestep_x, double& basestep_y) const;

  /// The parameter for the discretization
  DiscreteResolution res_;

  /// Base Index of represented base
  BaseIndex base_idx_;

  /// The rotation of the basestep.
  double dyaw_;
  double dpitch_;

  std::vector<BasestepXY> backward_steps_;
  std::vector<BasestepXY> forward_steps_;

  double step_cost_;
};

L3_STATIC_ASSERT_MOVEABLE(Basestep)
}  // namespace l3_footstep_planning

#endif
