#include <ros/ros.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <l3_libs/robot_description/base_info.h>

#include <l3_footstep_planning_libs/macros.h>

#include <l3_footstep_planning_libs/modeling/foothold_database.h>
#include <l3_footstep_planning_libs/modeling/state_database.h>

#define STATE_ELEMENTS(idx) fh_arrays_[(idx)], fb_arrays_[(idx)]
#define STATE_ELEMENTS_PTR(idx) fh_ptr_arrays_[(idx)], fb_ptr_arrays_[(idx)]
#define STATE_ELEMENTS_HASHED_PTR(idx) fh_hashed_ptr_arrays_[(idx)], fb_hashed_ptr_arrays_[(idx)]

using namespace l3_footstep_planning;

class StateDatabaseFixture : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    res_ = DiscreteResolution(0.05, 0.05, 0.025, M_PI * 0.1);
    db_.reset(new StateDataBase(res_));
  }

  void SetUp() override
  {
    // generate footholds
    fh_.clear();
    fh_.push_back(Foothold(0, 0.0, 0.02, 0.01, 0.0, 0.0, 0.12));
    fh_.push_back(Foothold(1, 0.0, -0.1, -0.02, 0.0, 0.1, -0.05));
    fh_.push_back(Foothold(1, 0.1, -0.1, -0.02, 0.0, 0.1, -0.05));
    fh_.push_back(Foothold(2, 0.1, -0.1, 0.015, 0.0, 0.1, -0.05));
    fh_.push_back(Foothold(0, 0.01, 0.02, 0.01, 0.0, 0.0, 0.12));
    fh_.push_back(Foothold(1, 0.0, -0.1, -0.021, 0.0, 0.1, -0.05));
    fh_.push_back(Foothold(0, 0.02, 0.01, 0.0, 1.0, 2.0, 0.09));
    fh_.push_back(Foothold(1, 0.11, -0.12, -0.03, 0.1, 0.2, -0.06));

    // generate hashed footholds
    fh_ptr_.clear();
    fh_hashed_ptr_.clear();
    for (const Foothold& f : fh_)
    {
      FootholdID id(f, res_);
      fh_ptr_.push_back(makeShared<Foothold>(f));
      fh_hashed_ptr_.push_back(FootholdHashed::make(id.getDiscreteFoothold(res_), id));
    }

    // generate floating base
    fb_.clear();
    fb_.push_back(FloatingBase(0, 0.2, 0.1, -0.5, 1.1, 1.0, -0.4));
    fb_.push_back(FloatingBase(3, 0.15, 0.5, 1.5, 0.1, 0.0, 0.1));
    fb_.push_back(FloatingBase(0, 0.21, 0.12, -0.5, 1.1, 1.0, -0.41));
    fb_.push_back(FloatingBase(3, 0.14, 0.5, 1.5, 0.1, 0.1, 0.11));

    // generate hashed floating base
    fb_ptr_.clear();
    fb_hashed_ptr_.clear();
    for (const FloatingBase& f : fb_)
    {
      FloatingBaseID id(f, res_);
      fb_ptr_.push_back(makeShared<FloatingBase>(f));
      fb_hashed_ptr_.push_back(FloatingBaseHashed::make(id.getDiscreteFloatingBase(res_), id));
    }

    // generate states
    fh_arrays_.clear();
    fh_ptr_arrays_.clear();
    fh_hashed_ptr_arrays_.clear();
    states_.clear();

    generateState(0, 1); // 0: state #1
    generateState(0, 2); // 1: state #2
    generateState(0, 3); // 2: will never be added to db
    generateState(4, 5); // 3: similar to state #1
    generateState(6, 7); // 4: similar to state #2

    generateState(0, 1, 0); // 5: state #3
    generateState(0, 2, 1); // 6: state #4
    generateState(0, 4, 0); // 7: will never be added to db
    generateState(4, 5, 2); // 8: similar to state #3
    generateState(4, 7, 3); // 9: similar to state #4

    // these values can be differ depending how far the test suite has been progressed
    states_ptr_.clear();
    for (const State& s : states_)
      states_ptr_.push_back(db_->get(s));
  }

  void TearDown() override
  {
    fh_.clear();
    fh_ptr_.clear();
    fh_hashed_ptr_.clear();
    fh_arrays_.clear();
    fh_ptr_arrays_.clear();
    fh_hashed_ptr_arrays_.clear();

    fb_.clear();
    fb_ptr_.clear();
    fb_hashed_ptr_.clear();
    fb_arrays_.clear();
    fb_ptr_arrays_.clear();
    fb_hashed_ptr_arrays_.clear();

    states_.clear();
    states_ptr_.clear();
  }

  void generateState(unsigned long idx1, unsigned long idx2)
  {
    fh_arrays_.push_back(FootholdArray({fh_[idx1], fh_[idx2]}));
    fh_ptr_arrays_.push_back(FootholdConstPtrArray({fh_ptr_[idx1], fh_ptr_[idx2]}));
    fh_hashed_ptr_arrays_.push_back(FootholdHashedConstPtrArray({fh_hashed_ptr_[idx1], fh_hashed_ptr_[idx2]}));

    fb_arrays_.push_back(FloatingBaseArray());
    fb_ptr_arrays_.push_back(FloatingBaseConstPtrArray());
    fb_hashed_ptr_arrays_.push_back(FloatingBaseHashedConstPtrArray());

    states_.push_back(State(fh_hashed_ptr_arrays_.back(), fb_hashed_ptr_arrays_.back()));
  }

  void generateState(unsigned long fh_idx1, unsigned long fh_idx2, unsigned long fb_idx)
  {
    fh_arrays_.push_back(FootholdArray({fh_[fh_idx1], fh_[fh_idx2]}));
    fh_ptr_arrays_.push_back(FootholdConstPtrArray({fh_ptr_[fh_idx1], fh_ptr_[fh_idx2]}));
    fh_hashed_ptr_arrays_.push_back(FootholdHashedConstPtrArray({fh_hashed_ptr_[fh_idx1], fh_hashed_ptr_[fh_idx2]}));

    fb_arrays_.push_back(FloatingBaseArray({fb_[fb_idx]}));
    fb_ptr_arrays_.push_back(FloatingBaseConstPtrArray({fb_ptr_[fb_idx]}));
    fb_hashed_ptr_arrays_.push_back(FloatingBaseHashedConstPtrArray({fb_hashed_ptr_[fb_idx]}));

    states_.push_back(State(fh_hashed_ptr_arrays_.back(), fb_hashed_ptr_arrays_.back()));
  }

  FootholdArray fh_;
  FootholdConstPtrArray fh_ptr_;
  FootholdHashedConstPtrArray fh_hashed_ptr_;

  std::vector<FootholdArray> fh_arrays_;
  std::vector<FootholdConstPtrArray> fh_ptr_arrays_;
  std::vector<FootholdHashedConstPtrArray> fh_hashed_ptr_arrays_;

  FloatingBaseArray fb_;
  FloatingBaseConstPtrArray fb_ptr_;
  FloatingBaseHashedConstPtrArray fb_hashed_ptr_;

  std::vector<FloatingBaseArray> fb_arrays_;
  std::vector<FloatingBaseConstPtrArray> fb_ptr_arrays_;
  std::vector<FloatingBaseHashedConstPtrArray> fb_hashed_ptr_arrays_;

  std::vector<State> states_;
  std::vector<StateHashed::ConstPtr> states_ptr_;

  static DiscreteResolution res_;
  static StateDataBase::Ptr db_;
};

DiscreteResolution StateDatabaseFixture::res_ = DiscreteResolution();
StateDataBase::Ptr StateDatabaseFixture::db_ = StateDataBase::Ptr();

TEST_F(StateDatabaseFixture, State)
{
  FootholdArray fh;
  fh.push_back(Foothold(0, 0.0, 0.02, -0.06, 0.0, 0.0, 0.12));
  fh.push_back(Foothold(1, 0.0, -0.1, 0.03, 0.0, 0.1, -0.05));

  FootholdHashedConstPtrArray fh_ptr;

  FootholdID id = FootholdID(fh[0], res_);
  fh_ptr.push_back(FootholdHashed::make(id.getDiscreteFoothold(res_), id));

  id = FootholdID(fh[1], res_);
  fh_ptr.push_back(FootholdHashed::make(id.getDiscreteFoothold(res_), id));

  StateHashed::ConstPtr s_ptr = StateHashed::make(State(fh_ptr, FloatingBaseHashedConstPtrArray()), StateID(fh_ptr, FloatingBaseHashedConstPtrArray()));

  // check footholds
  EXPECT_EQ(2, s_ptr->getFootholds().size());
  EXPECT_EQ(fh_ptr[0], s_ptr->getFoothold(0));
  EXPECT_EQ(fh_ptr[1], s_ptr->getFoothold(1));
}

TEST_F(StateDatabaseFixture, Insert)
{
  // insert first state
  StateHashed::ConstPtr s1_ptr = db_->insert(states_[0]);
  EXPECT_EQ(1, db_->size());

  // double insertion should have no effect
  EXPECT_EQ(s1_ptr.get(), db_->insert(states_[0]).get());
  EXPECT_EQ(1, db_->size());

  // insert second state
  db_->insert(State::ConstPtr(new State(states_[1])));
  EXPECT_EQ(2, db_->size());

  // insert third state
  StateHashed::ConstPtr s3_ptr = db_->insert(State::ConstPtr(new State(states_[5])));
  EXPECT_EQ(3, db_->size());

  // double insertion should have no effect
  EXPECT_EQ(s3_ptr.get(), db_->insert(states_[5]).get());
  EXPECT_EQ(3, db_->size());

  // insert fourth state
  db_->insert(State::ConstPtr(new State(states_[6])));
  EXPECT_EQ(4, db_->size());
}

TEST_F(StateDatabaseFixture, Has)
{
  // check for empty matches
  EXPECT_DB_ENTRY_EQ(State::ConstPtr(), *db_, states_[2]);
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS(2)));
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS_PTR(2)));
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS_HASHED_PTR(2)));


  // check 'has' method
  EXPECT_TRUE(db_->has(states_ptr_[0]->getUID()));
  EXPECT_TRUE(db_->has(states_ptr_[1]->getUID()));

  // state #1
  EXPECT_TRUE(db_->has(StateKey(STATE_ELEMENTS(0))));
  EXPECT_TRUE(db_->has(STATE_ELEMENTS(0)));
  EXPECT_TRUE(db_->has(STATE_ELEMENTS_PTR(0)));
  EXPECT_TRUE(db_->has(STATE_ELEMENTS_HASHED_PTR(0)));
  EXPECT_TRUE(db_->has(states_[0]));

  // state #2
  EXPECT_TRUE(db_->has(StateKey(STATE_ELEMENTS(1))));
  EXPECT_TRUE(db_->has(STATE_ELEMENTS(1)));
  EXPECT_TRUE(db_->has(STATE_ELEMENTS_PTR(1)));
  EXPECT_TRUE(db_->has(STATE_ELEMENTS_HASHED_PTR(1)));
  EXPECT_TRUE(db_->has(states_[1]));

  // non exisiting
  EXPECT_FALSE(db_->has(StateKey(STATE_ELEMENTS(2))));
  EXPECT_FALSE(db_->has(STATE_ELEMENTS(2)));
  EXPECT_FALSE(db_->has(STATE_ELEMENTS_PTR(2)));
  EXPECT_FALSE(db_->has(STATE_ELEMENTS_HASHED_PTR(2)));
  EXPECT_FALSE(db_->has(states_[2]));

  // state #3
  EXPECT_TRUE(db_->has(StateKey(STATE_ELEMENTS(5))));
  EXPECT_TRUE(db_->has(STATE_ELEMENTS(5)));
  EXPECT_TRUE(db_->has(STATE_ELEMENTS_PTR(5)));
  EXPECT_TRUE(db_->has(STATE_ELEMENTS_HASHED_PTR(5)));
  EXPECT_TRUE(db_->has(states_[5]));

  // state #4
  EXPECT_TRUE(db_->has(StateKey(STATE_ELEMENTS(6))));
  EXPECT_TRUE(db_->has(STATE_ELEMENTS(6)));
  EXPECT_TRUE(db_->has(STATE_ELEMENTS_PTR(6)));
  EXPECT_TRUE(db_->has(STATE_ELEMENTS_HASHED_PTR(6)));
  EXPECT_TRUE(db_->has(states_[6]));

  // non exisiting
  EXPECT_FALSE(db_->has(StateKey(STATE_ELEMENTS(7))));
  EXPECT_FALSE(db_->has(STATE_ELEMENTS(7)));
  EXPECT_FALSE(db_->has(STATE_ELEMENTS_PTR(7)));
  EXPECT_FALSE(db_->has(STATE_ELEMENTS_HASHED_PTR(7)));
  EXPECT_FALSE(db_->has(states_[7]));
}

TEST_F(StateDatabaseFixture, GetOperators)
{
  // check exact matches
  // state #1
  EXPECT_EQ(states_ptr_[0], db_->get(STATE_ELEMENTS(0)));
  EXPECT_EQ(states_ptr_[0], db_->get(STATE_ELEMENTS_PTR(0)));
  EXPECT_EQ(states_ptr_[0], db_->get(STATE_ELEMENTS_HASHED_PTR(0)));
  EXPECT_EQ(states_ptr_[0], (*db_)(STATE_ELEMENTS(0)));
  EXPECT_EQ(states_ptr_[0], (*db_)(STATE_ELEMENTS_PTR(0)));
  EXPECT_EQ(states_ptr_[0], (*db_)(STATE_ELEMENTS_HASHED_PTR(0)));

  // state #2
  EXPECT_EQ(states_ptr_[1], db_->get(STATE_ELEMENTS(1)));
  EXPECT_EQ(states_ptr_[1], db_->get(STATE_ELEMENTS_PTR(1)));
  EXPECT_EQ(states_ptr_[1], db_->get(STATE_ELEMENTS_HASHED_PTR(1)));
  EXPECT_EQ(states_ptr_[1], (*db_)(STATE_ELEMENTS(1)));
  EXPECT_EQ(states_ptr_[1], (*db_)(STATE_ELEMENTS_PTR(1)));
  EXPECT_EQ(states_ptr_[1], (*db_)(STATE_ELEMENTS_HASHED_PTR(1)));

  // state #3
  EXPECT_EQ(states_ptr_[5], db_->get(STATE_ELEMENTS(5)));
  EXPECT_EQ(states_ptr_[5], db_->get(STATE_ELEMENTS_PTR(5)));
  EXPECT_EQ(states_ptr_[5], db_->get(STATE_ELEMENTS_HASHED_PTR(5)));
  EXPECT_EQ(states_ptr_[5], (*db_)(STATE_ELEMENTS(5)));
  EXPECT_EQ(states_ptr_[5], (*db_)(STATE_ELEMENTS_PTR(5)));
  EXPECT_EQ(states_ptr_[5], (*db_)(STATE_ELEMENTS_HASHED_PTR(5)));

  // state #4
  EXPECT_EQ(states_ptr_[6], db_->get(STATE_ELEMENTS(6)));
  EXPECT_EQ(states_ptr_[6], db_->get(STATE_ELEMENTS_PTR(6)));
  EXPECT_EQ(states_ptr_[6], db_->get(STATE_ELEMENTS_HASHED_PTR(6)));
  EXPECT_EQ(states_ptr_[6], (*db_)(STATE_ELEMENTS(6)));
  EXPECT_EQ(states_ptr_[6], (*db_)(STATE_ELEMENTS_PTR(6)));
  EXPECT_EQ(states_ptr_[6], (*db_)(STATE_ELEMENTS_HASHED_PTR(6)));
}

TEST_F(StateDatabaseFixture, UID)
{
  // check uid system
  EXPECT_EQ(1, states_ptr_[0]->getUID());
  EXPECT_EQ(2, states_ptr_[1]->getUID());
  EXPECT_EQ(3, states_ptr_[5]->getUID());
  EXPECT_EQ(4, states_ptr_[6]->getUID());

  EXPECT_DB_ENTRY_EQ(states_ptr_[0], *db_, states_ptr_[0]->getUID());
  EXPECT_DB_ENTRY_EQ(states_ptr_[1], *db_, states_ptr_[1]->getUID());
  EXPECT_DB_ENTRY_EQ(states_ptr_[5], *db_, states_ptr_[5]->getUID());
  EXPECT_DB_ENTRY_EQ(states_ptr_[6], *db_, states_ptr_[6]->getUID());
}

TEST_F(StateDatabaseFixture, Similarity)
{
  // check for similar matches (due to binning)
  // state 3 -> 0
  EXPECT_EQ(states_ptr_[0], db_->get(STATE_ELEMENTS(3)));
  EXPECT_EQ(states_ptr_[0], db_->get(STATE_ELEMENTS_PTR(3)));
  EXPECT_EQ(states_ptr_[0], db_->get(STATE_ELEMENTS_HASHED_PTR(3)));

  // state 4 -> 1
  EXPECT_EQ(states_ptr_[1], db_->get(STATE_ELEMENTS(4)));
  EXPECT_EQ(states_ptr_[1], db_->get(STATE_ELEMENTS_PTR(4)));
  EXPECT_EQ(states_ptr_[1], db_->get(STATE_ELEMENTS_HASHED_PTR(4)));

  // state 8 -> 5
  EXPECT_EQ(states_ptr_[5], db_->get(STATE_ELEMENTS(8)));
  EXPECT_EQ(states_ptr_[5], db_->get(STATE_ELEMENTS_PTR(8)));
  EXPECT_EQ(states_ptr_[5], db_->get(STATE_ELEMENTS_HASHED_PTR(8)));

  // state 9 -> 6
  EXPECT_EQ(states_ptr_[6], db_->get(STATE_ELEMENTS(9)));
  EXPECT_EQ(states_ptr_[6], db_->get(STATE_ELEMENTS_PTR(9)));
  EXPECT_EQ(states_ptr_[6], db_->get(STATE_ELEMENTS_HASHED_PTR(9)));

  // insertion of similar states should return existing one
  // state 3 -> 0
  EXPECT_EQ(states_ptr_[0].get(), db_->insert(State(STATE_ELEMENTS_HASHED_PTR(3))).get());
  EXPECT_EQ(states_ptr_[0], db_->get(STATE_ELEMENTS(0)));
  EXPECT_EQ(4, db_->size());

  // state 4 -> 1
  EXPECT_EQ(states_ptr_[1].get(), db_->insert(State(STATE_ELEMENTS_HASHED_PTR(4))).get());
  EXPECT_EQ(states_ptr_[1], db_->get(STATE_ELEMENTS(1)));
  EXPECT_EQ(4, db_->size());

  // state 8 -> 5
  EXPECT_EQ(states_ptr_[5].get(), db_->insert(State(STATE_ELEMENTS_HASHED_PTR(8))).get());
  EXPECT_EQ(states_ptr_[5], db_->get(STATE_ELEMENTS(5)));
  EXPECT_EQ(4, db_->size());

  // state 9 -> 6
  EXPECT_EQ(states_ptr_[6].get(), db_->insert(State(STATE_ELEMENTS_HASHED_PTR(9))).get());
  EXPECT_EQ(states_ptr_[6], db_->get(STATE_ELEMENTS(6)));
  EXPECT_EQ(4, db_->size());
}

TEST_F(StateDatabaseFixture, Consistency)
{
  // check consistency of inserted elements
  EXPECT_EQ(Foothold(*fh_hashed_ptr_[0]), Foothold(*states_ptr_[0]->getFoothold(0)));
  EXPECT_EQ(Foothold(*fh_hashed_ptr_[4]), Foothold(*states_ptr_[0]->getFoothold(0)));
  EXPECT_EQ(Foothold(*fh_hashed_ptr_[1]), Foothold(*states_ptr_[0]->getFoothold(1)));
  EXPECT_EQ(Foothold(*fh_hashed_ptr_[5]), Foothold(*states_ptr_[0]->getFoothold(1)));
  EXPECT_EQ(FloatingBase::ConstPtr(), states_ptr_[0]->getFloatingBase(BaseInfo::MAIN_BODY_IDX));

  EXPECT_EQ(Foothold(*fh_hashed_ptr_[0]), Foothold(*states_ptr_[1]->getFoothold(0)));
  EXPECT_EQ(Foothold(*fh_hashed_ptr_[6]), Foothold(*states_ptr_[1]->getFoothold(0)));
  EXPECT_EQ(Foothold(*fh_hashed_ptr_[2]), Foothold(*states_ptr_[1]->getFoothold(1)));
  EXPECT_EQ(Foothold(*fh_hashed_ptr_[7]), Foothold(*states_ptr_[1]->getFoothold(1)));
  EXPECT_EQ(FloatingBase::ConstPtr(), states_ptr_[1]->getFloatingBase(BaseInfo::MAIN_BODY_IDX));

  EXPECT_EQ(Foothold(*fh_hashed_ptr_[0]), Foothold(*states_ptr_[5]->getFoothold(0)));
  EXPECT_EQ(Foothold(*fh_hashed_ptr_[4]), Foothold(*states_ptr_[5]->getFoothold(0)));
  EXPECT_EQ(Foothold(*fh_hashed_ptr_[1]), Foothold(*states_ptr_[5]->getFoothold(1)));
  EXPECT_EQ(Foothold(*fh_hashed_ptr_[5]), Foothold(*states_ptr_[5]->getFoothold(1)));
  EXPECT_EQ(FloatingBase(*fb_hashed_ptr_[0]), FloatingBase(*states_ptr_[5]->getFloatingBase(BaseInfo::MAIN_BODY_IDX)));
  EXPECT_EQ(FloatingBase(*fb_hashed_ptr_[2]), FloatingBase(*states_ptr_[5]->getFloatingBase(BaseInfo::MAIN_BODY_IDX)));

  EXPECT_EQ(Foothold(*fh_hashed_ptr_[0]), Foothold(*states_ptr_[6]->getFoothold(0)));
  EXPECT_EQ(Foothold(*fh_hashed_ptr_[6]), Foothold(*states_ptr_[6]->getFoothold(0)));
  EXPECT_EQ(Foothold(*fh_hashed_ptr_[2]), Foothold(*states_ptr_[6]->getFoothold(1)));
  EXPECT_EQ(Foothold(*fh_hashed_ptr_[7]), Foothold(*states_ptr_[6]->getFoothold(1)));
  EXPECT_EQ(FloatingBase(*fb_hashed_ptr_[1]), FloatingBase(*states_ptr_[6]->getFloatingBase(3)));
  EXPECT_EQ(FloatingBase(*fb_hashed_ptr_[3]), FloatingBase(*states_ptr_[6]->getFloatingBase(3)));
}

TEST_F(StateDatabaseFixture, Remove)
{
  // check remove
  db_->remove(STATE_ELEMENTS(4));
  EXPECT_EQ(3, db_->size());

  EXPECT_EQ(states_ptr_[0], db_->get(STATE_ELEMENTS(3)));

  EXPECT_DB_ENTRY_EQ(State::ConstPtr(), *db_, states_ptr_[1]->getUID());
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS(1)));
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS_PTR(1)));
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS_HASHED_PTR(1)));

  EXPECT_DB_ENTRY_EQ(State::ConstPtr(), *db_, states_ptr_[4]->getUID());
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS(4)));
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS_PTR(4)));
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS_HASHED_PTR(4)));

  // check remove
  db_->remove(STATE_ELEMENTS_PTR(9));
  EXPECT_EQ(2, db_->size());

  EXPECT_EQ(states_ptr_[5], db_->get(STATE_ELEMENTS_PTR(8)));

  EXPECT_EQ(State::ConstPtr(), db_->get(states_ptr_[6]->getUID()));
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS(6)));
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS_PTR(6)));
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS_HASHED_PTR(6)));

  EXPECT_EQ(State::ConstPtr(), db_->get(states_ptr_[9]->getUID()));
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS(9)));
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS_PTR(9)));
  EXPECT_EQ(State::ConstPtr(), db_->get(STATE_ELEMENTS_HASHED_PTR(9)));

  // removing non-existing element should have no effect
  db_->remove(fh_arrays_[2], FloatingBaseArray());
  db_->remove(fh_hashed_ptr_arrays_[7], fb_hashed_ptr_arrays_[0]);
  EXPECT_EQ(2, db_->size());
}

TEST_F(StateDatabaseFixture, Readd)
{
  // check re-adding
  StateHashed::ConstPtr s2_new_ptr = db_->insert(states_[1]);
  EXPECT_NE(states_ptr_[1], s2_new_ptr);
  EXPECT_EQ(3, db_->size());
  EXPECT_EQ(5, s2_new_ptr->getUID());
  EXPECT_DB_ENTRY_EQ(s2_new_ptr, *db_, UID(5));
  EXPECT_DB_DOUBLE_ENTRY_EQ(s2_new_ptr, *db_, states_[1].getFootholdsHashed(), states_[1].getFloatingBasesHashed());
}

TEST_F(StateDatabaseFixture, Clear)
{
  // check clear
  EXPECT_FALSE(db_->empty());
  db_->clear();
  EXPECT_EQ(0, db_->size());
  EXPECT_TRUE(db_->empty());
}
