#include <ros/ros.h>

#include <gtest/gtest.h>

#include <l3_footstep_planning_libs/macros.h>

#include <l3_footstep_planning_libs/modeling/foothold_database.h>
#include <l3_footstep_planning_libs/modeling/state_database.h>

#include <l3_footstep_planning_libs/modeling/transition_database.h>

using namespace l3_footstep_planning;

TEST(Typedefs, Transition)
{
  DiscreteResolution res(0.05, 0.05, 0.025, M_PI * 0.1);
  FootholdDataBase db(res);
  StateDataBase state_db(res);
  TransitionDataBase trans_db;

  FootholdArray fh;
  fh.push_back(Foothold(0, 0.0, 0.02, -0.06, 0.0, 0.0, 0.12));
  fh.push_back(Foothold(1, 0.0, -0.1, 0.03, 0.0, 0.1, -0.05));
  fh.push_back(Foothold(0, 0.0, 0.02, -0.06, 0.0, 0.0, 0.12));
  fh.push_back(Foothold(1, 0.1, -0.1, 0.03, 0.0, 0.1, -0.05));

  FootholdHashedConstPtrArray fh1_ptr;
  fh1_ptr.push_back(db.insert(fh[0]));
  fh1_ptr.push_back(db.insert(fh[1]));
  StateHashed::ConstPtr s1_ptr = state_db.insert(State(fh1_ptr, FloatingBaseHashedConstPtrArray()));

  FootholdHashedConstPtrArray fh2_ptr;
  fh2_ptr.push_back(db.insert(fh[2]));
  fh2_ptr.push_back(db.insert(fh[3]));
  StateHashed::ConstPtr s2_ptr = state_db.insert(State(fh2_ptr, FloatingBaseHashedConstPtrArray()));

  Transition t(FootStep::ConstPtr(), s1_ptr, s2_ptr, 0.2, 0.1);
  TransitionHashed::ConstPtr t_ptr = trans_db.insert(t);

  // check footholds
  EXPECT_EQ(s1_ptr, t_ptr->getPredecessor());
  EXPECT_EQ(s2_ptr, t_ptr->getSuccessor());
  EXPECT_DOUBLE_EQ(0.2, t_ptr->getCost());
  EXPECT_DOUBLE_EQ(0.1, t_ptr->getRisk());
}

TEST(Typedefs, TransitionID)
{
  DiscreteResolution res(0.05, 0.05, 0.025, M_PI * 0.1);
  FootholdDataBase db(res);

  // test clean initalization
  EXPECT_STATE_ID(Eigen::VectorXi(), StateID(FootholdHashedConstPtrArray(), FloatingBaseHashedConstPtrArray()));

  FootholdArray fh1;
  fh1.push_back(Foothold(0, 0.0, 0.02, -0.06, 0.0, 0.0, 0.12));
  fh1.push_back(Foothold(1, 0.0, -0.1, 0.03, 0.0, 0.1, -0.05));
  StateID s_id1(fh1, FloatingBaseArray(), res);

  FootholdArray fh2;
  fh2.push_back(Foothold(0, 0.0, 0.02, -0.06, 0.0, 0.0, 0.12));
  fh2.push_back(Foothold(1, 0.1, -0.1, 0.03, 0.0, 0.1, -0.05));
  StateID s_id2(fh2, FloatingBaseArray(), res);

  TransitionID t_id(s_id1, s_id2);

  // major binning tests
  Eigen::VectorXi exp1(10);
  exp1 << 0, 0, 0, -2, 0, 1, 0, -2, 1, 0;
  EXPECT_STATE_ID(exp1, t_id.getPredecessorID());

  Eigen::VectorXi exp2(10);
  exp2 << 0, 0, 0, -2, 0, 1, 2, -2, 1, 0;
  EXPECT_STATE_ID(exp2, t_id.getSuccessorID());
}

TEST(Typedefs, TransitionHashing)
{
  DiscreteResolution res(0.05, 0.05, 0.025, M_PI * 0.1);
  FootholdDataBase db(res);
  StateDataBase state_db(res);
  TransitionDataBase trans_db;

  // generate transition
  FootholdArray fh;
  fh.push_back(Foothold(0, 0.0, 0.02, -0.06, 0.0, 0.0, 0.12));
  fh.push_back(Foothold(1, 0.0, -0.1, 0.03, 0.0, 0.1, -0.05));
  fh.push_back(Foothold(0, 0.0, 0.02, -0.06, 0.0, 0.0, 0.12));
  fh.push_back(Foothold(1, 0.1, -0.1, 0.03, 0.0, 0.1, -0.05));

  FootholdHashedConstPtrArray fh1_ptr;
  fh1_ptr.push_back(db.insert(fh[0]));
  fh1_ptr.push_back(db.insert(fh[1]));
  StateID s1_id(fh1_ptr, FloatingBaseHashedConstPtrArray());
  StateHashed::ConstPtr s1_ptr = state_db.insert(State(fh1_ptr, FloatingBaseHashedConstPtrArray()));

  FootholdHashedConstPtrArray fh2_ptr;
  fh2_ptr.push_back(db.insert(fh[2]));
  fh2_ptr.push_back(db.insert(fh[3]));
  StateID s2_id(fh2_ptr, FloatingBaseHashedConstPtrArray());
  StateHashed::ConstPtr s2_ptr = state_db.insert(State(fh2_ptr, FloatingBaseHashedConstPtrArray()));

  Transition t(FootStep::ConstPtr(), s1_ptr, s2_ptr, 0.2, 0.1);
  TransitionHashed::ConstPtr t_ptr = trans_db.insert(t);

  // check if id still matches
  EXPECT_EQ(s1_ptr->getID(), t_ptr->getPredecessor()->getID());
  EXPECT_EQ(s2_ptr->getID(), t_ptr->getSuccessor()->getID());
  EXPECT_EQ(TransitionID(s1_ptr->getID(), s2_ptr->getID()), t_ptr->getID());

  // check if db insertion keeps states' hash
  EXPECT_EQ(s1_id.getHashValue(), t_ptr->getPredecessor()->getHashValue());
  EXPECT_EQ(s2_id.getHashValue(), t_ptr->getSuccessor()->getHashValue());

  // check transition hash computation based on cached state hashes
  Hash h = 0;
  boost::hash_combine(h, s1_ptr->getHashValue());
  boost::hash_combine(h, s2_ptr->getHashValue());
  EXPECT_EQ(h, t_ptr->getHashValue());
}

TEST(Database, TransitionDataBase)
{
  /**
   * Modeled graph:
   *
   *   --t1-->(s2)--t2-->(s3)
   *  |        ^
   * (s1)      t4
   *  |        |
   *   --t3-->(s4)
   */

  DiscreteResolution res(0.05, 0.05, 0.025, M_PI * 0.1);
  FootholdDataBase foothold_db(res);
  StateDataBase state_db(res);
  TransitionDataBase trans_db;

  FootholdArray fh;
  fh.push_back(Foothold(0, 0.0, 0.02, 0.01, 0.0, 0.0, 0.12));
  fh.push_back(Foothold(1, 0.0, -0.1, -0.02, 0.0, 0.1, -0.05));
  fh.push_back(Foothold(0, 0.0, 0.02, -0.06, 0.0, 0.0, 0.12));
  fh.push_back(Foothold(1, 0.1, -0.1, 0.03, 0.0, 0.1, -0.05));

  // generate hashed footholds
  FootholdHashedConstPtrArray fh_ptr;
  for (const Foothold& f : fh)
    fh_ptr.push_back(foothold_db.insert(f));

  // generate hashed states
  FootholdHashedConstPtrArray fh1_ptr;
  fh1_ptr.push_back(fh_ptr[0]);
  fh1_ptr.push_back(fh_ptr[1]);
  StateHashed::ConstPtr s1_ptr = state_db.insert(State(fh1_ptr, FloatingBaseHashedConstPtrArray()));
  StateID s1_id = s1_ptr->getID();

  FootholdHashedConstPtrArray fh2_ptr;
  fh2_ptr.push_back(fh_ptr[2]);
  fh2_ptr.push_back(fh_ptr[3]);
  StateHashed::ConstPtr s2_ptr = state_db.insert(State(fh2_ptr, FloatingBaseHashedConstPtrArray()));
  StateID s2_id = s2_ptr->getID();

  FootholdHashedConstPtrArray fh3_ptr;
  fh3_ptr.push_back(fh_ptr[0]);
  fh3_ptr.push_back(fh_ptr[3]);
  StateHashed::ConstPtr s3_ptr = state_db.insert(State(fh3_ptr, FloatingBaseHashedConstPtrArray()));
  StateID s3_id = s3_ptr->getID();

  FootholdHashedConstPtrArray fh4_ptr;
  fh4_ptr.push_back(fh_ptr[2]);
  fh4_ptr.push_back(fh_ptr[1]);
  StateHashed::ConstPtr s4_ptr = state_db.insert(State(fh4_ptr, FloatingBaseHashedConstPtrArray()));
  StateID s4_id = s4_ptr->getID();

  // insert first transition
  TransitionID t1_id(s1_id, s2_id);
  Transition t1(FootStep::ConstPtr(), s1_ptr, s2_ptr, 0.2, 0.1);
  TransitionHashed::ConstPtr t1_ptr = trans_db.insert(t1);

  EXPECT_EQ(1, trans_db.size());

  // double insertion should have no effect
  EXPECT_EQ(t1_ptr.get(), trans_db.insert(t1).get());
  EXPECT_EQ(1, trans_db.size());
  EXPECT_EQ(1, trans_db.getPredecessors(s2_id).size());
  EXPECT_EQ(1, trans_db.getSuccessors(s1_id).size());

  // insert second transition
  TransitionID t2_id(s2_id, s3_id);
  Transition t2(FootStep::ConstPtr(), s2_ptr, s3_ptr, 0.14, 0.01);
  TransitionHashed::ConstPtr t2_ptr = trans_db.insert(t2);

  EXPECT_EQ(2, trans_db.size());

  // insert third transition
  TransitionID t3_id(s1_id, s4_id);
  Transition t3(FootStep::ConstPtr(), s1_ptr, s4_ptr, 0.05, 0.11);
  TransitionHashed::ConstPtr t3_ptr = trans_db.insert(t3);

  EXPECT_EQ(3, trans_db.size());

  // insert fourth transition
  TransitionID t4_id(s4_id, s2_id);
  Transition t4(FootStep::ConstPtr(), s4_ptr, s2_ptr, 0.4, 0.1);
  TransitionHashed::ConstPtr t4_ptr = trans_db.insert(t4);

  EXPECT_EQ(4, trans_db.size());

  // check for empty matches
  EXPECT_DB_ENTRY_EQ(Transition::ConstPtr(), trans_db, TransitionID(s2_id, s1_id));
  EXPECT_EQ(Transition::ConstPtr(), trans_db(s2_id, s1_id));
  EXPECT_EQ(Transition::ConstPtr(), trans_db.get(s2_id, s1_id));
  EXPECT_DB_ENTRY_EQ(Transition::ConstPtr(), trans_db, TransitionID(s3_id, s2_id));
  EXPECT_EQ(Transition::ConstPtr(), trans_db(s3_id, s2_id));
  EXPECT_EQ(Transition::ConstPtr(), trans_db.get(s3_id, s2_id));

  // check 'has' method
  EXPECT_TRUE(trans_db.has(t1_id));
  EXPECT_TRUE(trans_db.has(s1_id, s2_id));
  EXPECT_TRUE(trans_db.has(t2_id));
  EXPECT_TRUE(trans_db.has(s2_id, s3_id));
  EXPECT_TRUE(trans_db.has(t3_id));
  EXPECT_TRUE(trans_db.has(s1_id, s4_id));
  EXPECT_TRUE(trans_db.has(t4_id));
  EXPECT_TRUE(trans_db.has(s4_id, s2_id));
  EXPECT_FALSE(trans_db.has(TransitionID(s3_id, s2_id)));
  EXPECT_FALSE(trans_db.has(s4_id, s1_id));

  // check getter
  EXPECT_DB_ENTRY_EQ(t1_ptr, trans_db, t1_id);
  EXPECT_EQ(t1_ptr, trans_db(s1_id, s2_id));
  EXPECT_EQ(t1_ptr, trans_db.get(s1_id, s2_id));

  EXPECT_DB_ENTRY_EQ(t2_ptr, trans_db, t2_id);
  EXPECT_EQ(t2_ptr, trans_db(s2_id, s3_id));
  EXPECT_EQ(t2_ptr, trans_db.get(s2_id, s3_id));

  EXPECT_DB_ENTRY_EQ(t3_ptr, trans_db, t3_id);
  EXPECT_EQ(t3_ptr, trans_db(s1_id, s4_id));
  EXPECT_EQ(t3_ptr, trans_db.get(s1_id, s4_id));

  EXPECT_DB_ENTRY_EQ(t4_ptr, trans_db, t4_id);
  EXPECT_EQ(t4_ptr, trans_db(s4_id, s2_id));
  EXPECT_EQ(t4_ptr, trans_db.get(s4_id, s2_id));

  // check predecessors mapping
  EXPECT_EQ(0, trans_db.getPredecessors(s1_id).size());
  EXPECT_EQ(2, trans_db.getPredecessors(s2_id).size());
  EXPECT_EQ(1, trans_db.getPredecessors(s3_id).size());
  EXPECT_EQ(1, trans_db.getPredecessors(s4_id).size());

  EXPECT_EQ(t1_ptr, *(trans_db.getPredecessors(s2_id).find(t1_ptr)));
  EXPECT_EQ(t4_ptr, *(trans_db.getPredecessors(s2_id).find(t4_ptr)));
  EXPECT_EQ(t2_ptr, *(trans_db.getPredecessors(s3_id).begin()));
  EXPECT_EQ(t3_ptr, *(trans_db.getPredecessors(s4_id).begin()));

  // check successor mapping
  EXPECT_EQ(2, trans_db.getSuccessors(s1_id).size());
  EXPECT_EQ(1, trans_db.getSuccessors(s2_id).size());
  EXPECT_EQ(0, trans_db.getSuccessors(s3_id).size());
  EXPECT_EQ(1, trans_db.getSuccessors(s4_id).size());

  EXPECT_EQ(t1_ptr, *(trans_db.getSuccessors(s1_id).find(t1_ptr)));
  EXPECT_EQ(t3_ptr, *(trans_db.getSuccessors(s1_id).find(t3_ptr)));
  EXPECT_EQ(t2_ptr, *(trans_db.getSuccessors(s2_id).begin()));
  EXPECT_EQ(t4_ptr, *(trans_db.getSuccessors(s4_id).begin()));

  // check remove
  trans_db.remove(t1_id);

  EXPECT_EQ(3, trans_db.size());

  EXPECT_DB_ENTRY_EQ(Transition::ConstPtr(), trans_db, t1_id);
  EXPECT_EQ(Transition::ConstPtr(), trans_db(s1_id, s2_id));
  EXPECT_EQ(Transition::ConstPtr(), trans_db.get(s1_id, s2_id));

  EXPECT_DB_ENTRY_EQ(t2_ptr, trans_db, t2_id);

  // check graph after removal
  EXPECT_EQ(0, trans_db.getPredecessors(s1_id).size());
  EXPECT_EQ(1, trans_db.getPredecessors(s2_id).size());
  EXPECT_EQ(1, trans_db.getPredecessors(s3_id).size());
  EXPECT_EQ(1, trans_db.getPredecessors(s4_id).size());

  EXPECT_EQ(t4_ptr, *(trans_db.getPredecessors(s2_id).begin()));
  EXPECT_EQ(t2_ptr, *(trans_db.getPredecessors(s3_id).begin()));
  EXPECT_EQ(t3_ptr, *(trans_db.getPredecessors(s4_id).begin()));

  EXPECT_EQ(1, trans_db.getSuccessors(s1_id).size());
  EXPECT_EQ(1, trans_db.getSuccessors(s2_id).size());
  EXPECT_EQ(0, trans_db.getSuccessors(s3_id).size());
  EXPECT_EQ(1, trans_db.getSuccessors(s4_id).size());

  EXPECT_EQ(t3_ptr, *(trans_db.getSuccessors(s1_id).begin()));
  EXPECT_EQ(t2_ptr, *(trans_db.getSuccessors(s2_id).begin()));
  EXPECT_EQ(t4_ptr, *(trans_db.getSuccessors(s4_id).begin()));

  // check re-adding
  TransitionHashed::ConstPtr t1_new_ptr = trans_db.insert(t1);
  EXPECT_NE(t1_ptr, t1_new_ptr);
  EXPECT_EQ(4, trans_db.size());
  EXPECT_DB_ENTRY_EQ(t1_new_ptr, trans_db, t1_id);

  // check clear
  EXPECT_FALSE(trans_db.empty());
  trans_db.clear();
  EXPECT_EQ(0, trans_db.size());
  EXPECT_TRUE(trans_db.empty());
}
