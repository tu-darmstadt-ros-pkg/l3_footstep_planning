#include <l3_footstep_planning_libs/msgs_test_helper.h>

#include <gtest/gtest.h>

namespace l3_footstep_planning
{
#define EXPECT_DOUBLE_NEAR(expected, actual, abs_error)                                                                                                                            \
  if (abs_error == 0.0)                                                                                                                                                            \
    EXPECT_DOUBLE_EQ(expected, actual);                                                                                                                                            \
  else                                                                                                                                                                             \
    EXPECT_NEAR(expected, actual, abs_error);

void isEqualTest(const std_msgs::String& exp, const std_msgs::String& res) { EXPECT_EQ(exp.data, res.data); }

void isEqualTest(const std_msgs::Header& exp, const std_msgs::Header& res)
{
  EXPECT_EQ(exp.seq, res.seq);
  EXPECT_EQ(exp.stamp, res.stamp);
  EXPECT_EQ(exp.frame_id, res.frame_id);
}

void isEqualTest(const geometry_msgs::Point& exp, const geometry_msgs::Point& res, double abs_error)
{
  EXPECT_DOUBLE_NEAR(exp.x, res.x, abs_error);
  EXPECT_DOUBLE_NEAR(exp.y, res.y, abs_error);
  EXPECT_DOUBLE_NEAR(exp.z, res.z, abs_error);
}

void isEqualTest(const geometry_msgs::Quaternion& exp, const geometry_msgs::Quaternion& res, double abs_error)
{
  EXPECT_DOUBLE_NEAR(exp.x, res.x, abs_error);
  EXPECT_DOUBLE_NEAR(exp.y, res.y, abs_error);
  EXPECT_DOUBLE_NEAR(exp.z, res.z, abs_error);
  EXPECT_DOUBLE_NEAR(exp.w, res.w, abs_error);
}

void isEqualTest(const geometry_msgs::Pose& exp, const geometry_msgs::Pose& res, double abs_error)
{
  isEqualTest(exp.position, res.position, abs_error);
  isEqualTest(exp.orientation, res.orientation, abs_error);
}

void isEqualTest(const msgs::Foothold& exp, const msgs::Foothold& res, double abs_error)
{
  isEqualTest(exp.header, res.header);
  EXPECT_EQ(exp.idx, res.idx);
  isEqualTest(exp.pose, res.pose, abs_error);
}

void isEqualTest(const msgs::FloatingBase& exp, const msgs::FloatingBase& res, double abs_error)
{
  isEqualTest(exp.header, res.header);
  EXPECT_EQ(exp.idx, res.idx);
  isEqualTest(exp.pose, res.pose, abs_error);
}

void isEqualTest(const msgs::FootholdArray& exp, const msgs::FootholdArray& res, double abs_error)
{
  ASSERT_EQ(exp.size(), res.size());
  for (size_t i = 0; i < exp.size(); i++)
    isEqualTest(exp[i], res[i], abs_error);
}

void isEqualTest(const msgs::FloatingBaseArray& exp, const msgs::FloatingBaseArray& res, double abs_error)
{
  ASSERT_EQ(exp.size(), res.size());
  for (size_t i = 0; i < exp.size(); i++)
    isEqualTest(exp[i], res[i], abs_error);
}

void isEqualTest(const msgs::FootStepData& exp, const msgs::FootStepData& res, double abs_error)
{
  isEqualTest(exp.origin, res.origin, abs_error);
  isEqualTest(exp.target, res.target, abs_error);

  EXPECT_DOUBLE_NEAR(exp.dx, res.dx, abs_error);
  EXPECT_DOUBLE_NEAR(exp.dy, res.dy, abs_error);
  EXPECT_DOUBLE_NEAR(exp.dz, res.dz, abs_error);
  EXPECT_DOUBLE_NEAR(exp.droll, res.droll, abs_error);
  EXPECT_DOUBLE_NEAR(exp.dpitch, res.dpitch, abs_error);
  EXPECT_DOUBLE_NEAR(exp.dyaw, res.dyaw, abs_error);
  EXPECT_DOUBLE_NEAR(exp.sway_duration, res.sway_duration, abs_error);
  EXPECT_DOUBLE_NEAR(exp.step_duration, res.step_duration, abs_error);
  EXPECT_DOUBLE_NEAR(exp.swing_height, res.swing_height, abs_error);
}

void isEqualTest(const msgs::BaseStepData& exp, const msgs::BaseStepData& res, double abs_error)
{
  isEqualTest(exp.origin, res.origin, abs_error);
  isEqualTest(exp.target, res.target, abs_error);

  EXPECT_DOUBLE_NEAR(exp.dx, res.dx, abs_error);
  EXPECT_DOUBLE_NEAR(exp.dy, res.dy, abs_error);
  EXPECT_DOUBLE_NEAR(exp.dz, res.dz, abs_error);
  EXPECT_DOUBLE_NEAR(exp.droll, res.droll, abs_error);
  EXPECT_DOUBLE_NEAR(exp.dpitch, res.dpitch, abs_error);
  EXPECT_DOUBLE_NEAR(exp.dyaw, res.dyaw, abs_error);
  EXPECT_DOUBLE_NEAR(exp.step_duration, res.step_duration, abs_error);
}

void isEqualTest(const msgs::Step& exp, const msgs::Step& res, double abs_error)
{
  EXPECT_EQ(exp.idx, res.idx);

  for (size_t i = 0; i < exp.foot_steps.size(); i++)
    isEqualTest(exp.foot_steps[i], res.foot_steps[i], abs_error);
  isEqualTest(exp.support_feet, res.support_feet, abs_error);

  for (size_t i = 0; i < exp.moving_bases.size(); i++)
    isEqualTest(exp.moving_bases[i], res.moving_bases[i], abs_error);
  isEqualTest(exp.resting_bases, res.resting_bases, abs_error);
}

void isEqualTest(const msgs::StepPlan& exp, const msgs::StepPlan& res, double abs_error)
{
  isEqualTest(exp.header, res.header);
  isEqualTest(exp.parameter_set_name, res.parameter_set_name);
  isEqualTest(exp.start, res.start, abs_error);
  isEqualTest(exp.goal, res.goal, abs_error);

  ASSERT_EQ(exp.plan.steps.size(), res.plan.steps.size());
  for (size_t i = 0; i < exp.plan.steps.size(); i++)
    isEqualTest(exp.plan.steps[i], res.plan.steps[i], abs_error);
}

void isEqualTest(const StepPlan& exp, const StepPlan& res, double abs_error)
{
  msgs::StepPlan exp_steps;
  exp.toMsg(exp_steps);

  msgs::StepPlan res_steps;
  res.toMsg(res_steps);

  isEqualTest(exp_steps, res_steps, abs_error);
}
}  // namespace l3_footstep_planning
