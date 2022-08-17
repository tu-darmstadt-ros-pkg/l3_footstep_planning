#include <ros/ros.h>

#include <math.h>

#include <gtest/gtest.h>

#include <l3_footstep_planning_libs/msgs_test_helper.h>

using namespace l3_footstep_planning;

/**
 * @brief Generates pattern of steps
 * @param start_left Start pose for left foot
 * @param start_right Start pose for right foot
 * @param steps Number of steps (initial step and closing step excluded)
 * @param foot_idx First foot to move (0 = Left, 1 = Right)
 * @param start_step_idx Starting step index from which to count up; If 0 then starting step will be prepended.
 * @param closing_step True, when closing step should be generated
 * @return
 */
msgs::StepPlan genStepPlan(geometry_msgs::Pose start_left, geometry_msgs::Pose start_right, unsigned int steps, l3::FootIndex foot_idx = 0, l3::StepIndex start_step_idx = 0,
                           bool closing_step = true)
{
  struct State
  {
    msgs::Foothold left, right;
  } state;

  double step_distance = 0.5;

  bool first_step = start_step_idx == 0;

  double yaw = tf::getYaw(foot_idx == 0 ? start_left.orientation : start_right.orientation);

  msgs::StepPlan step_plan;

  // init state
  state.left.idx = 0;
  state.left.pose = start_left;
  state.right.idx = 1;
  state.right.pose = start_right;

  // generate initial step
  msgs::FootStepData foot_step;
  if (foot_idx == 0)
  {
    foot_step.origin.idx = foot_step.target.idx = 1;
    foot_step.origin.pose = foot_step.target.pose = start_right;
  }
  else
  {
    foot_step.origin.idx = foot_step.target.idx = 0;
    foot_step.origin.pose = foot_step.target.pose = start_left;
  }

  msgs::Step step;
  step.idx = start_step_idx;

  if (first_step)
  {
    step.support_feet.push_back(state.left);
    step.support_feet.push_back(state.right);
    step_plan.plan.steps.push_back(step);
    step.idx++;

    step_distance /= 2.0;
  }

  // add extra step when closing step is set
  if (closing_step)
    steps++;

  for (size_t i = 0; i < steps; i++)
  {
    // closing step handling
    if (closing_step && i == steps - 1)
    {
      if (foot_step.origin.idx == 1)
      {
        foot_step.origin = state.left;
        state.left.pose.position.x = state.right.pose.position.x + 0.2 * -sin(yaw);
        state.left.pose.position.y = state.right.pose.position.y + 0.2 * cos(yaw);
        foot_step.target = state.left;
      }
      else
      {
        foot_step.origin = state.right;
        state.right.pose.position.x = state.left.pose.position.x - 0.2 * -sin(yaw);
        state.right.pose.position.y = state.left.pose.position.y - 0.2 * cos(yaw);
        foot_step.target = state.right;
      }
    }
    // forward movement
    else
    {
      // alternate left and right foot pose
      if (foot_step.origin.idx == 1)
      {
        foot_step.origin = state.left;
        state.left.pose.position.x += step_distance * cos(yaw);
        state.left.pose.position.y += step_distance * sin(yaw);
        foot_step.target = state.left;
      }
      else
      {
        foot_step.origin = state.right;
        state.right.pose.position.x += step_distance * cos(yaw);
        state.right.pose.position.y += step_distance * sin(yaw);
        foot_step.target = state.right;
      }
    }

    step.support_feet.clear();
    if (foot_step.origin.idx == 1)
      step.support_feet.push_back(state.left);
    else
      step.support_feet.push_back(state.right);

    step.foot_steps.clear();
    step.foot_steps.push_back(foot_step);

    step_plan.plan.steps.push_back(step);
    step.idx++;

    if (first_step)
    {
      step_distance *= 2.0;
      first_step = false;
    }
  }
  return step_plan;
}

msgs::StepPlan genStepPlan(geometry_msgs::Pose start, unsigned int steps, const l3::FootIndex& foot_idx = 0, const l3::StepIndex& start_step_idx = 0, bool closing_step = true)
{
  geometry_msgs::Pose start_left = start;
  geometry_msgs::Pose start_right = start;

  double yaw = tf::getYaw(start.orientation);

  start_left.position.x += 0.1 * -sin(yaw);
  start_left.position.y += 0.1 * cos(yaw);

  start_right.position.x -= 0.1 * -sin(yaw);
  start_right.position.y -= 0.1 * cos(yaw);

  return genStepPlan(start_left, start_right, steps, foot_idx, start_step_idx, closing_step);
}

// Test for checking if appendStepPlan generates correct results
TEST(StepPlan, appendStepPlan)
{
  geometry_msgs::Pose start;
  start.position.x = 1.0;
  start.position.y = 0.1;
  start.position.z = 1.0;
  start.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // generate plan 1: 5 steps, no closing step
  msgs::StepPlan step_plan_1 = genStepPlan(start, 5, 0, 0, false);
  ASSERT_EQ(6, step_plan_1.plan.steps.size());

  // generate plan 2: 6 steps + closing step, starting from last step of step_plan_1
  msgs::StepPlan step_plan_2 = genStepPlan(step_plan_1.plan.steps[5].foot_steps[0].target.pose, step_plan_1.plan.steps[4].foot_steps[0].target.pose, 6, 1, 6);
  ASSERT_EQ(7, step_plan_2.plan.steps.size());

  // append plan 1 and 2
  StepPlan step_plan_result(step_plan_1);
  step_plan_result.appendStepPlan(step_plan_2);
  ASSERT_EQ(13, step_plan_result.size());

  // generate expected step plan
  StepPlan step_plan_exp(genStepPlan(start, 11));
  ASSERT_EQ(13, step_plan_exp.size());

  // compare
  isEqualTest(step_plan_exp, step_plan_result);
}

// Test for checking if updateStepPlan generates correct results
TEST(StepPlan, updateStepPlan)
{
  geometry_msgs::Pose start;
  start.position.x = 1.0;
  start.position.y = 0.1;
  start.position.z = 1.0;
  start.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // generate plan 1: 6 steps
  msgs::StepPlan step_plan_1 = genStepPlan(start, 6, 0);

  // generate plan 2: 8 steps, starting from step #3 of step_plan_1
  msgs::StepPlan step_plan_2 = genStepPlan(step_plan_1.plan.steps[3].foot_steps.front().target.pose, step_plan_1.plan.steps[4].foot_steps.front().target.pose, 6, 0, 5);

  // update plan 1 with 2
  StepPlan step_plan_result(step_plan_1);
  step_plan_result.updateStepPlan(step_plan_2);
  ASSERT_EQ(12, step_plan_result.size());

  // generate expected step plan
  StepPlan step_plan_exp(genStepPlan(start, 10));
  ASSERT_EQ(12, step_plan_exp.size());

  for (l3::StepIndex step_idx = 3; step_idx < 8; step_idx++)
    EXPECT_FALSE(step_plan_exp.getStep(step_idx) == nullptr);

  // compare
  isEqualTest(step_plan_exp, step_plan_result);
}

// Test for checking if stitchStepPlan generates correct results
TEST(StepPlan, stitchStepPlan1)
{
  geometry_msgs::Pose start_1;
  start_1.position.x = 1.0;
  start_1.position.y = 1.1;
  start_1.position.z = 0.0;
  start_1.orientation = tf::createQuaternionMsgFromYaw(M_PI_4);

  // generate plan 1
  msgs::StepPlan step_plan_1 = genStepPlan(start_1, 4);

  geometry_msgs::Pose start_2;
  start_2.position.x = 2.1;
  start_2.position.y = 1.0;
  start_2.position.z = 1.0;
  start_2.orientation = tf::createQuaternionMsgFromYaw(1.0);

  // generate plan 2
  msgs::StepPlan step_plan_2 = genStepPlan(start_2, 8);

  // stitch plan 1 with 2
  StepPlan step_plan_result(step_plan_1);
  step_plan_result.stitchStepPlan(step_plan_2, 3);
  ASSERT_EQ(10, step_plan_result.size());

  // generate expected step plan
  StepPlan step_plan_exp(genStepPlan(start_1, 8));
  ASSERT_EQ(10, step_plan_exp.size());

  for (l3::StepIndex step_idx = 3; step_idx < 8; step_idx++)
    EXPECT_FALSE(step_plan_exp.getStep(step_idx) == nullptr);

  // compare
  isEqualTest(step_plan_exp, step_plan_result, 10e-15);
}

// Test for checking if stitchStepPlan generates correct results (reversal case of stitchStepPlan1)
TEST(StepPlan, stitchStepPlan2)
{
  geometry_msgs::Pose start_1;
  start_1.position.x = 2.1;
  start_1.position.y = 1.0;
  start_1.position.z = 1.0;
  start_1.orientation = tf::createQuaternionMsgFromYaw(1.0);

  // generate plan 1
  msgs::StepPlan step_plan_1 = genStepPlan(start_1, 4);

  geometry_msgs::Pose start_2;
  start_2.position.x = 1.0;
  start_2.position.y = 1.1;
  start_2.position.z = 0.0;
  start_2.orientation = tf::createQuaternionMsgFromYaw(M_PI_4);

  // generate plan 2
  msgs::StepPlan step_plan_2 = genStepPlan(start_2, 8);

  // stitch plan 1 with 2
  StepPlan step_plan_result(step_plan_1);
  step_plan_result.stitchStepPlan(step_plan_2, 3);
  ASSERT_EQ(10, step_plan_result.size());

  // generate expected step plan
  StepPlan step_plan_exp(genStepPlan(start_1, 8));
  ASSERT_EQ(10, step_plan_exp.size());

  for (l3::StepIndex step_idx = 3; step_idx < 8; step_idx++)
    EXPECT_FALSE(step_plan_exp.getStep(step_idx) == nullptr);

  // compare
  isEqualTest(step_plan_exp, step_plan_result, 10e-15);
}

TEST(StepPlan, transform)
{
  geometry_msgs::Pose start;
  start.position.x = 1.0;
  start.position.y = 2.0;
  start.position.z = 3.0;
  start.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // generate test plan
  StepPlan step_plan(genStepPlan(start, 4));
  step_plan.transform(Transform(-1.0, -2.0, -3.0, 0.0, 0.0, 0.0));

  // generate expected step plan
  start.position.x = 0.0;
  start.position.y = 0.0;
  start.position.z = 0.0;
  start.orientation = tf::createQuaternionMsgFromYaw(0.0);
  StepPlan step_plan_exp(genStepPlan(start, 4));

  // compare
  isEqualTest(step_plan_exp, step_plan, 10e-15);
}
