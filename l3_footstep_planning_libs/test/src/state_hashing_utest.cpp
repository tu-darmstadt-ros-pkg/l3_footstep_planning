#include <ros/ros.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <l3_libs/robot_description/base_info.h>

#include <l3_footstep_planning_libs/macros.h>

#include <l3_footstep_planning_libs/modeling/state_id.h>

using namespace l3_footstep_planning;

class StateHashingFixture : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    res_ = DiscreteResolution(0.05, 0.05, 0.025, M_PI * 0.1);
  }

  void SetUp() override
  {
    // create footholds
    fh_.clear();
    fh_.push_back(Foothold(0, 0.0, 0.02, -0.06, 0.0, 0.0, 0.12));
    fh_.push_back(Foothold(1, 0.0, -0.1, 0.03, 0.0, 0.1, -0.05));

    fh_ptr_.clear();
    for (const Foothold& fh : fh_)
      fh_ptr_.push_back(makeShared<Foothold>(fh));

    fh_hashed_ptr_.clear();
    for (const Foothold& fh : fh_)
    {
      FootholdID id(fh, res_);
      fh_hashed_ptr_.push_back(FootholdHashed::make(id.getDiscreteFoothold(res_), id));
    }

    // create floating base
    fb_.clear();
    fb_.push_back(FloatingBase(2, 1.0, 2.0, -3.0, 1.2, 3.4, -1.24));

    fb_ptr_.clear();
    for (const FloatingBase& fb : fb_)
      fb_ptr_.push_back(makeShared<FloatingBase>(fb));

    fb_hashed_ptr_.clear();
    for (const FloatingBase& fb : fb_)
    {
      FloatingBaseID id(fb, res_);
      fb_hashed_ptr_.push_back(FloatingBaseHashed::make(id.getDiscreteFloatingBase(res_), id));
    }
  }

  void TearDown() override
  {
    fh_.clear();
    fh_ptr_.clear();
    fh_hashed_ptr_.clear();

    fb_.clear();
    fb_ptr_.clear();
    fb_hashed_ptr_.clear();
  }

  static DiscreteResolution res_;

  FootholdArray fh_;
  FootholdConstPtrArray fh_ptr_;
  FootholdHashedConstPtrArray fh_hashed_ptr_;

  FloatingBaseArray fb_;
  FloatingBaseConstPtrArray fb_ptr_;
  FloatingBaseHashedConstPtrArray fb_hashed_ptr_;
};

DiscreteResolution StateHashingFixture::res_ = DiscreteResolution();

TEST_F(StateHashingFixture, StateKeyNoBase)
{
  // test clean initalization
  StateKey key;
  EXPECT_EQ(0, key.footholds.size());
  EXPECT_EQ(0, key.floating_bases.size());

  // test constructors
  ASSERT_THAT(StateKey(fh_, FloatingBaseArray()).footholds, ::testing::ElementsAreArray(fh_));
  EXPECT_EQ(0, StateKey(fh_, FloatingBaseArray()).floating_bases.size());
  ASSERT_THAT(StateKey(fh_ptr_, FloatingBaseConstPtrArray()).footholds, ::testing::ElementsAreArray(fh_));
  EXPECT_EQ(0, StateKey(fh_ptr_, FloatingBaseConstPtrArray()).floating_bases.size());
}

TEST_F(StateHashingFixture, StateKeyWithBase)
{
  // test constructors
  ASSERT_THAT(StateKey(fh_, fb_).footholds, ::testing::ElementsAreArray(fh_));
  EXPECT_TRUE(StateKey(fh_, fb_).floating_bases.size() > 0);
  EXPECT_POSE_EQ(StateKey(fh_, fb_).floating_bases[BaseInfo::MAIN_BODY_IDX].pose(), fb_[BaseInfo::MAIN_BODY_IDX].pose());

  ASSERT_THAT(StateKey(fh_ptr_, fb_ptr_).footholds, ::testing::ElementsAreArray(fh_));
  EXPECT_TRUE(StateKey(fh_ptr_, fb_ptr_).floating_bases.size() > 0);
  EXPECT_POSE_EQ(StateKey(fh_ptr_, fb_ptr_).floating_bases[BaseInfo::MAIN_BODY_IDX].pose(), fb_[BaseInfo::MAIN_BODY_IDX].pose());
}

TEST_F(StateHashingFixture, StateIDNoBase)
{
  // test clean initalization
  EXPECT_STATE_ID(Eigen::VectorXi(), StateID());
  EXPECT_STATE_ID(Eigen::VectorXi(), StateID(FootholdHashedConstPtrArray(), FloatingBaseHashedConstPtrArray()));

  // binning tests
  Eigen::VectorXi exp(10);
  exp << 0, 0, 0, -2, 0, 1, 0, -2, 1, 0;

  EXPECT_STATE_ID(exp, StateID(StateKey(fh_, FloatingBaseArray()), res_));
  EXPECT_STATE_ID(exp, StateID(StateKey(fh_ptr_, FloatingBaseConstPtrArray()), res_));
  EXPECT_STATE_ID(exp, StateID(fh_, FloatingBaseArray(), res_));
  EXPECT_STATE_ID(exp, StateID(fh_ptr_, FloatingBaseConstPtrArray(), res_));

  // check cached id computation
  EXPECT_STATE_ID(exp, StateID(fh_hashed_ptr_, FloatingBaseHashedConstPtrArray()));

  // test plain StateID initialization
  std::vector<EigenFootholdID> fh_vec;
  EigenFootholdID vec_id;
  vec_id << 0, 0, 0, -2, 0;
  fh_vec.push_back(vec_id);
  vec_id << 1, 0, -2, 1, 0;
  fh_vec.push_back(vec_id);
  EXPECT_STATE_ID(exp, StateID(fh_vec, std::vector<EigenFloatingBaseID>()));

  // test initialization using states
  EXPECT_STATE_ID(exp, StateID(State(fh_hashed_ptr_, FloatingBaseHashedConstPtrArray())));
}

TEST_F(StateHashingFixture, StateIDWithBase)
{
  // test clean initalization
  EXPECT_STATE_ID(Eigen::VectorXi(), StateID(FootholdHashedConstPtrArray(), FloatingBaseHashedConstPtrArray()));

  // binning tests
  Eigen::VectorXi exp(17);
  exp << 0, 0, 0, -2, 0, 1, 0, -2, 1, 0,
         2, 20, 40, -120, 4, -9, -4;

  EXPECT_STATE_ID(exp, StateID(StateKey(fh_, fb_), res_));
  EXPECT_STATE_ID(exp, StateID(StateKey(fh_ptr_, fb_ptr_), res_));
  EXPECT_STATE_ID(exp, StateID(fh_, fb_, res_));
  EXPECT_STATE_ID(exp, StateID(fh_ptr_, fb_ptr_, res_));

  EXPECT_FALSE(StateID(StateKey(fh_, FloatingBaseArray()), res_).size() == StateID(StateKey(fh_, fb_), res_).size());
  EXPECT_FALSE(StateID(StateKey(fh_ptr_, FloatingBaseConstPtrArray()), res_).size() == StateID(StateKey(fh_ptr_, fb_ptr_), res_).size());
  EXPECT_FALSE(StateID(fh_, FloatingBaseArray(), res_).size() == StateID(fh_, fb_, res_).size());
  EXPECT_FALSE(StateID(fh_ptr_, FloatingBaseConstPtrArray(), res_).size() == StateID(fh_ptr_, fb_ptr_, res_).size());

  // check cached id computation
  EXPECT_STATE_ID(exp, StateID(fh_hashed_ptr_, fb_hashed_ptr_));

  // test plain StateID initialization
  std::vector<EigenFootholdID> fh_vec;
  EigenFootholdID vec_id;
  vec_id << 0, 0, 0, -2, 0;
  fh_vec.push_back(vec_id);
  vec_id << 1, 0, -2, 1, 0;
  fh_vec.push_back(vec_id);

  EigenFloatingBaseID fb_vec;
  fb_vec << 2, 20, 40, -120, 4, -9, -4;

  EXPECT_STATE_ID(exp, StateID(fh_vec, std::vector<EigenFloatingBaseID>({fb_vec})));

  FloatingBaseHashedConstPtrArray test_array = State(fh_hashed_ptr_, fb_hashed_ptr_).getFloatingBasesHashed();

  // test initialization using states
  EXPECT_STATE_ID(exp, StateID(State(fh_hashed_ptr_, fb_hashed_ptr_)));
}

TEST_F(StateHashingFixture, StateHashingNoBase)
{
  // generate state
  StateID id(fh_, FloatingBaseArray(), res_);
  StateHashed::ConstPtr s_ptr = StateHashed::make(State(fh_hashed_ptr_, FloatingBaseHashedConstPtrArray()), id);

  // check if id still matches
  EXPECT_EQ(id, s_ptr->getID());

  // compare state hash computation of both implementations (using State and FootholdHashedConstPtrArray)
  EXPECT_EQ(s_ptr->getHashValue(), id.getHashValue());

  // check state hash computation based on cached foothold hashes
  Hash h = 0;
  boost::hash_combine(h, fh_hashed_ptr_[0]->getHashValue());
  boost::hash_combine(h, fh_hashed_ptr_[1]->getHashValue());
  EXPECT_EQ(h, s_ptr->getHashValue());
}

TEST_F(StateHashingFixture, StateHashingWithBase)
{
  // generate state
  StateID id(fh_, fb_, res_);
  StateHashed::ConstPtr s_ptr = StateHashed::make(State(fh_hashed_ptr_, fb_hashed_ptr_), id);

  // check if id still matches
  EXPECT_EQ(id, s_ptr->getID());

  // compare state hash computation of both implementations (using State and FootholdHashedConstPtrArray)
  EXPECT_EQ(s_ptr->getHashValue(), id.getHashValue());

  EXPECT_NE(s_ptr->getHashValue(), StateID(fh_, FloatingBaseArray(), res_).getHashValue());

  // check state hash computation based on cached foothold hashes
  Hash h = 0;
  boost::hash_combine(h, fh_hashed_ptr_[0]->getHashValue());
  boost::hash_combine(h, fh_hashed_ptr_[1]->getHashValue());
  boost::hash_combine(h, fb_hashed_ptr_[BaseInfo::MAIN_BODY_IDX]->getHashValue());
  EXPECT_EQ(h, s_ptr->getHashValue());
}
