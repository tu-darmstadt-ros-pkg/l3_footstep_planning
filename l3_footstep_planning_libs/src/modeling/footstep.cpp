#include <l3_footstep_planning_libs/modeling/footstep.h>

#include <l3_math/angles.h>

#include <l3_footstep_planning_libs/helper.h>

namespace l3_footstep_planning
{
Footstep::Footstep(const Pose& neutral_stance, const FootIndex& foot_idx, double dx, double dy, double dyaw, double step_cost, const DiscreteResolution& res)
  : res_(res)
  , foot_idx_(foot_idx)
  , dyaw_(neutral_stance.yaw() + dyaw)
  , step_cost_(step_cost)
  , backward_steps_(res_.numAngleBins())
  , forward_steps_(res_.numAngleBins())
{
  double footstep_x;
  double footstep_y;

  for (int global_theta = 0; global_theta < res_.numAngleBins(); global_theta++)
  {
    calculateBackwardStep(neutral_stance, res_.toContAngle(global_theta), dx, dy, footstep_x, footstep_y);
    backward_steps_[global_theta] = FootstepXY(footstep_x, footstep_y);

    calculateForwardStep(neutral_stance, res_.toContAngle(global_theta), dx, dy, footstep_x, footstep_y);
    forward_steps_[global_theta] = FootstepXY(footstep_x, footstep_y);
  }
}

Foothold::Ptr Footstep::getPredFoothold(const Pose& robot_pose) const
{
  int yaw = res_.toDiscAngle(robot_pose.yaw());

  // theta has to be in [0..ivNumAngleBins)
  while (yaw < 0)
    yaw += res_.numAngleBins();
  while (yaw >= res_.numAngleBins())
    yaw -= res_.numAngleBins();

  const FootstepXY& xy = backward_steps_[yaw];

  return makeShared<Foothold>(foot_idx_, robot_pose.x() + xy.first, robot_pose.y() + xy.second, robot_pose.z(), 0.0, 0.0, normalizeAngle(robot_pose.yaw() - dyaw_));
}

Foothold::Ptr Footstep::getPredFoothold(const State& current) const
{
  ROS_ASSERT(current.getFoothold(foot_idx_));
  return getPredFoothold(current.getFeetCenter());
}

Foothold::Ptr Footstep::getSuccFoothold(const Pose& robot_pose) const
{
  int yaw = res_.toDiscAngle(robot_pose.yaw());

  // theta has to be in [0..ivNumAngleBins)
  while (yaw < 0)
    yaw += res_.numAngleBins();
  while (yaw >= res_.numAngleBins())
    yaw -= res_.numAngleBins();

  const FootstepXY& xy = forward_steps_[yaw];

  return makeShared<Foothold>(foot_idx_, robot_pose.x() + xy.first, robot_pose.y() + xy.second, robot_pose.z(), 0.0, 0.0, normalizeAngle(robot_pose.yaw() + dyaw_));
}

Foothold::Ptr Footstep::getSuccFoothold(const State& current) const
{
  ROS_ASSERT(current.getFoothold(foot_idx_));
  return getSuccFoothold(current.getFeetCenter());
}

void Footstep::calculateBackwardStep(const Pose& neutral_stance, double global_theta, double dx, double dy, double& footstep_x, double& footstep_y) const
{
  calculateStep(global_theta, neutral_stance.x() - dx, neutral_stance.y() - dy, footstep_x, footstep_y);
}

void Footstep::calculateForwardStep(const Pose& neutral_stance, double global_theta, double dx, double dy, double& footstep_x, double& footstep_y) const
{
  calculateStep(global_theta, neutral_stance.x() + dx, neutral_stance.y() + dy, footstep_x, footstep_y);
}

void Footstep::calculateStep(double global_theta, double dx, double dy, double& footstep_x, double& footstep_y) const
{
  double theta_cos = cos(global_theta);
  double theta_sin = sin(global_theta);

  footstep_x = theta_cos * dx - theta_sin * dy;
  footstep_y = theta_sin * dx + theta_cos * dy;
}
}  // namespace l3_footstep_planning
