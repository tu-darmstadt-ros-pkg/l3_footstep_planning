#include <ros/ros.h>

#include <gtest/gtest.h>

#include <l3_math/math.h>

#include <l3_footstep_planning_libs/macros.h>

#include <l3_footstep_planning_libs/modeling/floating_base_id.h>

using namespace l3_footstep_planning;

TEST(Typedefs, FloatingBaseID)
{
  DiscreteResolution res(0.05, 0.05, 0.025, M_PI * 0.1);

  // test clean initalization
  EXPECT_FLOATING_BASE_ID(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, FloatingBaseID(FloatingBase(), res));

  // test discrete values
  FloatingBaseID id = FloatingBaseID(FloatingBase(1, 0.05, 0.12, 0.01, 0.0, -0.3, 0.6), res);

  FloatingBase disc_floating_base = id.getDiscreteFloatingBase(res);

  EXPECT_DOUBLE_EQ(0.05, disc_floating_base.x());
  EXPECT_DOUBLE_EQ(0.1, disc_floating_base.y());
  EXPECT_DOUBLE_EQ(0.0, disc_floating_base.z());
  EXPECT_DOUBLE_EQ(0.0, disc_floating_base.roll());
  EXPECT_DOUBLE_EQ(M_PI * -0.1, disc_floating_base.pitch());
  EXPECT_DOUBLE_EQ(M_PI * 0.2, disc_floating_base.yaw());

  // major binning tests
  EXPECT_FLOATING_BASE_ID(1, 1, 2, 0, 0, 0, 0, FloatingBaseID(FloatingBase(1, 0.05, 0.12, 0.0, 0.0, 0.0, 0.1), res));
  EXPECT_FLOATING_BASE_ID(2, -4, 1, 4, 0, -5, 10, FloatingBaseID(FloatingBase(2, -0.18, 0.049, 0.1, 0.1, -1.57, 3.14), res));
}

TEST(Typedefs, FloatingBaseHashing)
{
  DiscreteResolution res(0.05, 0.05, 0.025, M_PI * 0.1);

  // generate footholds
  FloatingBase base(1, 0.05, 0.12, 0.0, -0.6, 1.0, 2.1);
  FloatingBaseID id(base, res);

  FloatingBaseHashed::Ptr ptr = FloatingBaseHashed::make(id.getDiscreteFloatingBase(res), id);

  // test hash computation
  EXPECT_EQ(id.getHashValue(), ptr->getHashValue());

  std::vector<Hash> hashes = { 13537199649261162080u,
                               18297303017781972032u, 3663796792713958044u, 1600878694825884720u,
                               10262178778411536599u, 9871298466851410819u, 4386732711621280337u };

  Hash h = 0;
  for (size_t i = 0; i < FLOATING_BASE_ID_SIZE; i++)
  {
    boost::hash_combine(h, id(static_cast<long>(i)));
    EXPECT_EQ(hashes[i], h);
  }

  EXPECT_EQ(id.getHashValue(), h);
  EXPECT_EQ(ptr->getHashValue(), h);
}
