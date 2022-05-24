#include <ros/ros.h>

#include <gtest/gtest.h>

#include <l3_footstep_planning_libs/macros.h>
#include <l3_footstep_planning_libs/math.h>
#include <l3_footstep_planning_libs/typedefs.h>

using namespace l3_footstep_planning;

TEST(Math, Resolution)
{
  Resolution res(0.1, 0.2, 0.3, 0.4);
  EXPECT_DOUBLE_EQ(0.1, res.x);
  EXPECT_DOUBLE_EQ(0.2, res.y);
  EXPECT_DOUBLE_EQ(0.3, res.z);
  EXPECT_DOUBLE_EQ(0.4, res.angle);

  res = res.invert();
  EXPECT_DOUBLE_EQ(1.0 / 0.1, res.x);
  EXPECT_DOUBLE_EQ(1.0 / 0.2, res.y);
  EXPECT_DOUBLE_EQ(1.0 / 0.3, res.z);
  EXPECT_DOUBLE_EQ(1.0 / 0.4, res.angle);
}

TEST(Math, DiscreteResolution)
{
  // general tests
  DiscreteResolution res(0.1, 0.2, 0.3, 2.0 * M_PI / 72.0);
  EXPECT_EQ(72, res.numAngleBins());

  res = DiscreteResolution(0.1, 0.2, 0.3, 72);
  EXPECT_EQ(72, res.numAngleBins());

  EXPECT_DOUBLE_EQ(0.2, res.toContX(2));
  EXPECT_DOUBLE_EQ(0.4, res.toContY(2));
  EXPECT_DOUBLE_EQ(0.6, res.toContZ(2));
  EXPECT_DOUBLE_EQ(2.0 * M_PI / 72.0 * 2.0, res.toContAngle(2));

  EXPECT_DOUBLE_EQ(2, res.toDiscX(0.2));
  EXPECT_DOUBLE_EQ(2, res.toDiscY(0.43));
  EXPECT_DOUBLE_EQ(2, res.toDiscZ(0.55));
  EXPECT_DOUBLE_EQ(3, res.toDiscAngle(0.28));

  // border tests
  res = DiscreteResolution(0.1, 0.02, 0.3, 72);

  EXPECT_EQ(0, res.toDiscY(0.005));
  EXPECT_EQ(0, res.toDiscY(-0.005));

  EXPECT_EQ(1, res.toDiscY(0.02));
  EXPECT_EQ(-1, res.toDiscY(-0.02));

  EXPECT_EQ(13, res.toDiscY(0.255));
  EXPECT_EQ(-13, res.toDiscY(-0.255));

  EXPECT_DOUBLE_EQ(0.0, res.toContY(0));
  EXPECT_DOUBLE_EQ(0.02, res.toContY(1));
  EXPECT_DOUBLE_EQ(-0.02, res.toContY(-1));
  EXPECT_DOUBLE_EQ(0.26, res.toContY(13));
  EXPECT_DOUBLE_EQ(-0.26, res.toContY(-13));
}
