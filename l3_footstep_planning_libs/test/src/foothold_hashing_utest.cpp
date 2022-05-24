#include <ros/ros.h>

#include <gtest/gtest.h>

#include <l3_footstep_planning_libs/macros.h>

#include <l3_footstep_planning_libs/modeling/foothold_id.h>

using namespace l3_footstep_planning;

TEST(Typedefs, FootholdKey)
{
  // test clean init
  EXPECT_FOOTHOLD_KEY(0, 0.0, 0.0, 0.0, 0.0, FootholdKey());

  // test all constructor variants
  Pose p(1.2, -2.0, 42.0, 0.0, 0.0, 0.2);
  Foothold::ConstPtr fh_ptr = Foothold::ConstPtr(new Foothold(2, p));

  EXPECT_FOOTHOLD_KEY(2, 1.2, -2.0, 42.0, 0.2, FootholdKey(*fh_ptr));
  EXPECT_FOOTHOLD_KEY(2, 1.2, -2.0, 42.0, 0.2, FootholdKey(fh_ptr));
  EXPECT_FOOTHOLD_KEY(2, 1.2, -2.0, 42.0, 0.2, FootholdKey(2, p));
}

TEST(Typedefs, FootholdID)
{
  DiscreteResolution res(0.05, 0.05, 0.025, M_PI * 0.1);
  Pose p(0.05, 0.12, 0.01, 0.0, 0.1, 0.3);
  Foothold fh(1, p);

  // test clean initalization
  EXPECT_FOOTHOLD_ID(0, 0.0, 0.0, 0.0, 0.0, FootholdID(Foothold(), res));

  // test discretization
  FootholdID id = FootholdID(fh, res);

  Foothold disc_foothold = id.getDiscreteFoothold(res);

  EXPECT_DOUBLE_EQ(0.05, disc_foothold.x());
  EXPECT_DOUBLE_EQ(0.1, disc_foothold.y());
  EXPECT_DOUBLE_EQ(0.0, disc_foothold.z());
  EXPECT_DOUBLE_EQ(M_PI * 0.1, disc_foothold.yaw());

  // test all constructor variants
  EXPECT_FOOTHOLD_ID(1, 1, 2, 0, 1, FootholdID(FootholdKey(fh), res));
  EXPECT_FOOTHOLD_ID(1, 1, 2, 0, 1, FootholdID(fh, res));
  EXPECT_FOOTHOLD_ID(1, 1, 2, 0, 1, FootholdID(1, p, res));
  EXPECT_FOOTHOLD_ID(1, 1, 2, 0, 1, FootholdID(1, p.x(), p.y(), p.z(), p.yaw(), res));

  EigenFootholdID vec_id;
  vec_id << 1, 1, 2, 0, 1;
  EXPECT_FOOTHOLD_ID(1, 1, 2, 0, 1, FootholdID(vec_id));

  // major binning tests
  EXPECT_FOOTHOLD_ID(1, 1, 2, 0, 0, FootholdID(Foothold(1, 0.05, 0.12, 0.0, 0.0, 0.0, 0.1), res));
  EXPECT_FOOTHOLD_ID(2, -4, 1, 4,  10, FootholdID(Foothold(2, -0.18, 0.049, 0.1, 0.2, 0.3,  3.14), res));
  EXPECT_FOOTHOLD_ID(2, -4, 1, 4, -10, FootholdID(Foothold(2, -0.18, 0.049, 0.1, 0.2, 0.3, -3.14), res));
}

TEST(Typedefs, FootholdHashing)
{
  DiscreteResolution res(0.05, 0.05, 0.025, M_PI * 0.1);

  // generate footholds
  Foothold fh(1, 0.05, 0.12, 0.0, 0.0, 0.0, 0.1);
  FootholdID id(fh, res);
  FootholdHashed::ConstPtr fh_ptr = FootholdHashed::make(id.getDiscreteFoothold(res), id);

  // check if id still matches
  EXPECT_EQ(id, fh_ptr->getID());

  // test hash computation
  EXPECT_EQ(id.getHashValue(), fh_ptr->getHashValue());

  std::vector<Hash> hashes = { 13537199649261162080u, 18297303017781972032u, 3663796792713958044u, 1600878694825884720u, 8890724455821763412u };

  Hash h = 0;
  for (size_t i = 0; i < FOOTHOLD_ID_SIZE; i++)
  {
    boost::hash_combine(h, id(i));
    EXPECT_EQ(hashes[i], h);
  }

  EXPECT_EQ(id.getHashValue(), h);
  EXPECT_EQ(fh_ptr->getHashValue(), h);
}
