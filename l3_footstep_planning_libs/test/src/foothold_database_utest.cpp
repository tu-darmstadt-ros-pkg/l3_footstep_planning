#include <ros/ros.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <l3_math/math.h>

#include <l3_footstep_planning_libs/macros.h>

#include <l3_footstep_planning_libs/modeling/foothold_database.h>

using namespace l3_footstep_planning;

class FootholdDatabaseFixture : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    res_ = DiscreteResolution(0.05, 0.05, 0.025, M_PI * 0.1);
    db_.reset(new FootholdDataBase(res_));
  }

  void SetUp() override
  {
    fh1_ = Foothold(0, 0.05, 0.12, 0.0, 0.0, 0.0, 0.1);
    fh2_ = Foothold(1, -0.18, 0.049, 0.1, 0.2, 0.3, 3.14);

    key1_ = FootholdKey(fh1_);
    key2_ = FootholdKey(fh2_);

    similar_fh1_ = Foothold(0, 0.04, 0.1, 0.01, 0.0, 0.0, 0.09);
    similar_fh2_ = Foothold(1, -0.19, 0.051, 0.11, 0.2, 0.3, 3.13);

    similar_key1_ = FootholdKey(similar_fh1_);
    similar_key2_ = FootholdKey(similar_fh2_);

    no_key1_ = FootholdKey(0, 0.04, 0.1, 0.03, 0.09);

    // these values can be differ depending how far the test suite has been progressed
    fh1_ptr_ = db_->get(key1_);
    fh2_ptr_ = db_->get(key2_);
  }

  void TearDown() override
  {
  }

  Foothold fh1_;
  Foothold fh2_;

  Foothold::ConstPtr fh1_ptr_;
  Foothold::ConstPtr fh2_ptr_;

  FootholdKey key1_;
  FootholdKey key2_;

  Foothold similar_fh1_;
  Foothold similar_fh2_;

  FootholdKey similar_key1_;
  FootholdKey similar_key2_;

  FootholdKey no_key1_;

  static DiscreteResolution res_;
  static FootholdDataBase::Ptr db_;
};

DiscreteResolution FootholdDatabaseFixture::res_ = DiscreteResolution();
FootholdDataBase::Ptr FootholdDatabaseFixture::db_ = FootholdDataBase::Ptr();

TEST_F(FootholdDatabaseFixture, InsertElement)
{
  // db should be intially empty
  EXPECT_EQ(0, db_->size());

  // check if enough entries are in db
  Foothold::ConstPtr fh1_ptr = db_->insert(fh1_);
  EXPECT_EQ(1, db_->size());

  // double insertion should have no effect
  EXPECT_EQ(fh1_ptr.get(), db_->insert(fh1_).get());
  EXPECT_EQ(1, db_->size());

  Foothold::ConstPtr fh2_ptr_ = db_->insert(Foothold::ConstPtr(new Foothold(fh2_)));
  EXPECT_EQ(2, db_->size());
}

TEST_F(FootholdDatabaseFixture, InsertSimilar)
{
  // insertion of similar keys should return existing object
  EXPECT_EQ(fh1_ptr_.get(), db_->insert(similar_fh1_).get());
  EXPECT_DB_ENTRY_EQ(fh1_ptr_, *db_, key1_);
  EXPECT_EQ(2, db_->size());

  EXPECT_EQ(fh2_ptr_.get(), db_->insert(similar_fh2_).get());
  EXPECT_DB_ENTRY_EQ(fh2_ptr_, *db_, key2_);
  EXPECT_EQ(2, db_->size());
}

TEST_F(FootholdDatabaseFixture, Get)
{
  EXPECT_EQ(fh1_ptr_.get(), db_->get(key1_).get());
  EXPECT_EQ(fh2_ptr_.get(), db_->get(key2_).get());
  EXPECT_EQ(fh1_ptr_.get(), db_->get(similar_key1_).get());
  EXPECT_EQ(fh2_ptr_.get(), db_->get(similar_key2_).get());
  EXPECT_EQ(nullptr, db_->get(no_key1_).get());

  EXPECT_EQ(fh1_ptr_.get(), db_->get(fh1_).get());
  EXPECT_EQ(fh2_ptr_.get(), db_->get(fh2_).get());
  EXPECT_EQ(nullptr, db_->get(Foothold(1, 1.2, 3.0, 0.0, 0.0, 0.0, -0.1)).get());

  // check exact matches
  EXPECT_DB_ENTRY_EQ(fh1_ptr_, *db_, key1_);
  EXPECT_DB_ENTRY_EQ(fh2_ptr_, *db_, key2_);

  // check for similar key matches (due to binning)
  EXPECT_DB_ENTRY_EQ(fh1_ptr_, *db_, similar_key1_);
  EXPECT_DB_ENTRY_EQ(fh2_ptr_, *db_, similar_key2_);

  // check for empty matches
  EXPECT_DB_ENTRY_EQ(Foothold::ConstPtr(), *db_, no_key1_);
}

TEST_F(FootholdDatabaseFixture, Has)
{
  EXPECT_TRUE(db_->has(key1_));
  EXPECT_TRUE(db_->has(key2_));
  EXPECT_TRUE(db_->has(similar_key1_));
  EXPECT_TRUE(db_->has(similar_key2_));
  EXPECT_FALSE(db_->has(no_key1_));

  EXPECT_TRUE(db_->has(fh1_));
  EXPECT_TRUE(db_->has(fh2_));
  EXPECT_FALSE(db_->has(Foothold(1, 1.2, 3.0, 0.0, 0.0, 0.0, -0.1)));
}

TEST_F(FootholdDatabaseFixture, Discretization)
{
  EXPECT_DOUBLE_EQ(l3::pround(fh1_.x(), res_.toCont().x), fh1_ptr_->x());
  EXPECT_DOUBLE_EQ(l3::pround(fh1_.y(), res_.toCont().y), fh1_ptr_->y());
  EXPECT_DOUBLE_EQ(l3::pround(fh1_.z(), res_.toCont().z), fh1_ptr_->z());
  EXPECT_DOUBLE_EQ(l3::pround(fh1_.yaw(), res_.toCont().angle), fh1_ptr_->yaw());

  EXPECT_DOUBLE_EQ(l3::pround(fh2_.x(), res_.toCont().x), fh2_ptr_->x());
  EXPECT_DOUBLE_EQ(l3::pround(fh2_.y(), res_.toCont().y), fh2_ptr_->y());
  EXPECT_DOUBLE_EQ(l3::pround(fh2_.z(), res_.toCont().z), fh2_ptr_->z());
  EXPECT_DOUBLE_EQ(l3::pround(fh2_.yaw(), res_.toCont().angle), fh2_ptr_->yaw());
}

TEST_F(FootholdDatabaseFixture, Hashing)
{
  // generate footholds
  FootholdID id1(fh1_, res_);
  FootholdID id2(fh2_, res_);

  // check if db insertion keeps footholds' hash
  EXPECT_EQ(id1.getHashValue(), db_->get(fh1_)->getHashValue());
  EXPECT_EQ(id2.getHashValue(), db_->get(fh2_)->getHashValue());

  EXPECT_EQ(id1.getHashValue(), db_->get(similar_fh1_)->getHashValue());
  EXPECT_EQ(id2.getHashValue(), db_->get(similar_fh2_)->getHashValue());
}

TEST_F(FootholdDatabaseFixture, Remove)
{
  db_->remove(similar_key2_);
  EXPECT_EQ(1, db_->size());
  EXPECT_DB_ENTRY_EQ(fh1_ptr_, *db_, similar_key1_);
  EXPECT_DB_ENTRY_EQ(Foothold::ConstPtr(), *db_, key2_);
}

TEST_F(FootholdDatabaseFixture, Readd)
{
  Foothold::ConstPtr fh2_new_ptr = db_->insert(Foothold::ConstPtr(new Foothold(fh2_)));
  EXPECT_NE(fh2_ptr_, fh2_new_ptr);
  EXPECT_EQ(2, db_->size());
  EXPECT_DB_ENTRY_EQ(fh2_new_ptr, *db_, similar_key2_);
}

TEST_F(FootholdDatabaseFixture, Clear)
{
  db_->insert(fh2_);
  EXPECT_EQ(2, db_->size());
  EXPECT_FALSE(db_->empty());
  db_->clear();
  EXPECT_EQ(0, db_->size());
  EXPECT_TRUE(db_->empty());
}
