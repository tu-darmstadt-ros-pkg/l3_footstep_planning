#include <ros/ros.h>

#include <gtest/gtest.h>

#include <l3_footstep_planning_libs/macros.h>
#include <l3_footstep_planning_libs/helper.h>

using namespace l3_footstep_planning;

TEST(Helper, DiscretizationBasics)
{
  Resolution res(0.1, 0.02, 0.04, M_PI / 5.0);

  std_msgs::Header header;
  header.frame_id = "test";
  VariantDataSet data;
  data["test"] = 42;

  Pose pose(0.02, 0.0, 0.11, 0.24, 1.1, 2.7);
  Foothold fh(1, pose, header, data);
  FloatingBase fb(2, pose, header, data);

  Pose pose_d(0.0, 0.0, 0.12, 0.0, 2.0 * res.angle, 4.0 * res.angle);
  Foothold fh_d(fh.idx, pose_d, fh.header, fh.data);
  FloatingBase fb_d(fb.idx, pose_d, fb.header, fb.data);

  // check discretization
  EXPECT_POSE_EQ(pose_d, discretize(pose, res));
  EXPECT_FOOTHOLD_EQ(fh_d, discretize(fh, res));
  EXPECT_FLOATING_BASE_EQ(fb_d, discretize(fb, res));

  // ensure the identical result to ids
  fh = Foothold(1, pose);
  fh.setRPY(0.0, 0.0, fh.yaw()); // FootholdId does not store roll and pitch
  fb = FloatingBase(2, pose);

  FootholdID fh_id(fh, res);
  FloatingBaseID fb_id(fb, res);

  EXPECT_FOOTHOLD_EQ(fh_id.getDiscreteFoothold(res), discretize(fh, res));
  EXPECT_FLOATING_BASE_EQ(fb_id.getDiscreteFloatingBase(res), discretize(fb, res));
}
