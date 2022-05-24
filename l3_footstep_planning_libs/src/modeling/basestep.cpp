#include <l3_footstep_planning_libs/modeling/basestep.h>

#include <l3_libs/robot_description/base_info.h>

#include <l3_math/angles.h>

#include <l3_footstep_planning_libs/helper.h>

namespace l3_footstep_planning
{
Basestep::Basestep(const Pose& neutral_stance, const BaseIndex& base_idx, double dx, double dy, double dyaw, double dpitch, double step_cost, const DiscreteResolution& res)
  : res_(res)
  , base_idx_(base_idx)
  , dyaw_(neutral_stance.yaw() + dyaw)
  , dpitch_(neutral_stance.pitch() + dpitch)
  , step_cost_(step_cost)
  , backward_steps_(res_.numAngleBins())
  , forward_steps_(res_.numAngleBins())
{
  double basestep_x;
  double basestep_y;

  for (int global_theta = 0; global_theta < res_.numAngleBins(); global_theta++)
  {
    calculateBackwardStep(neutral_stance, res_.toContAngle(global_theta), dx, dy, basestep_x, basestep_y);
    backward_steps_[global_theta] = BasestepXY(basestep_x, basestep_y);

    calculateForwardStep(neutral_stance, res_.toContAngle(global_theta), dx, dy, basestep_x, basestep_y);
    forward_steps_[global_theta] = BasestepXY(basestep_x, basestep_y);
  }
}

FloatingBase::Ptr Basestep::getPredFloatingBase(const Pose& robot_pose) const
{
  int yaw = res_.toDiscAngle(robot_pose.yaw());

  // theta has to be in [0..ivNumAngleBins)
  while (yaw < 0)
    yaw += res_.numAngleBins();
  while (yaw >= res_.numAngleBins())
    yaw -= res_.numAngleBins();

  const BasestepXY& xy = backward_steps_[yaw];

  return FloatingBase::Ptr(new FloatingBase(base_idx_, robot_pose.x() + xy.first, robot_pose.y() + xy.second, robot_pose.z(), 0.0, 0.0, normalizeAngle(robot_pose.yaw() - dyaw_)));
}

FloatingBase::Ptr Basestep::getPredFloatingBase(const State& current) const
{
  ROS_ASSERT(current.hasFloatingBases());
  return getPredFloatingBase(current.getFloatingBase(BaseInfo::MAIN_BODY_IDX)->pose());
}

FloatingBase::Ptr Basestep::getSuccFloatingBase(const Pose& robot_pose) const
{
  int yaw = res_.toDiscAngle(robot_pose.yaw());

  // theta has to be in [0..ivNumAngleBins)
  while (yaw < 0)
    yaw += res_.numAngleBins();
  while (yaw >= res_.numAngleBins())
    yaw -= res_.numAngleBins();

  const BasestepXY& xy = forward_steps_[yaw];

  return FloatingBase::Ptr(new FloatingBase(base_idx_, robot_pose.x() + xy.first, robot_pose.y() + xy.second, robot_pose.z(), 0.0, 0.0, normalizeAngle(robot_pose.yaw() + dyaw_)));
}

FloatingBase::Ptr Basestep::getSuccFloatingBase(const State& current) const
{
  ROS_ASSERT(current.hasFloatingBases());
  return getSuccFloatingBase(current.getFloatingBase(BaseInfo::MAIN_BODY_IDX)->pose());
}

void Basestep::calculateBackwardStep(const Pose& neutral_stance, double global_theta, double dx, double dy, double& basestep_x, double& basestep_y) const
{
  calculateStep(global_theta, neutral_stance.x() - dx, neutral_stance.y() - dy, basestep_x, basestep_y);
}

void Basestep::calculateForwardStep(const Pose& neutral_stance, double global_theta, double dx, double dy, double& basestep_x, double& basestep_y) const
{
  calculateStep(global_theta, neutral_stance.x() + dx, neutral_stance.y() + dy, basestep_x, basestep_y);
}

void Basestep::calculateStep(double global_theta, double dx, double dy, double& basestep_x, double& basestep_y) const
{
  double theta_cos = cos(global_theta);
  double theta_sin = sin(global_theta);

  basestep_x = theta_cos * dx - theta_sin * dy;
  basestep_y = theta_sin * dx + theta_cos * dy;
}
}  // namespace l3_footstep_planning
