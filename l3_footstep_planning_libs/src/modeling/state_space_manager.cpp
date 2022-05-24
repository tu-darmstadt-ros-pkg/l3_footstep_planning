#include <l3_footstep_planning_libs/modeling/state_space_manager.h>

#include <sbpl/planners/planner.h>

namespace l3_footstep_planning
{
StateSpaceManager::StateSpaceManager()
  : hits_(0u)
  , miss_(0u)
  , state_ID2_index_mapping_(nullptr)
{}

bool StateSpaceManager::initialize(const vigir_generic_params::ParameterSet& params)
{
  bool result = true;

  mutableInstance().res_ = DiscreteResolution(params.getSubset("resolution"));

  UniqueLock lock(instance().mutex_);

  mutableInstance().foothold_db_.reset(new FootholdDataBase(instance().res_));
  mutableInstance().floating_base_db_.reset(new FloatingBaseDataBase(instance().res_));
  mutableInstance().state_db_.reset(new StateDataBase(instance().res_));
  mutableInstance().transition_db_.reset(new TransitionDataBase());

  return result;
}

void StateSpaceManager::setSbplEnvironment(std::vector<int*>& state_ID2_index_mapping)
{
  UniqueLock lock(instance().mutex_);
  mutableInstance().state_ID2_index_mapping_ = &state_ID2_index_mapping;

  if (instance().state_ID2_index_mapping_->empty())
  {
    // add dummy entry for id = 0
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    mutableInstance().state_ID2_index_mapping_->push_back(entry);

    for (int i = 0; i < NUMOFINDICES_STATEID2IND; i++)
      (*mutableInstance().state_ID2_index_mapping_)[0][i] = -1;
  }
}

void StateSpaceManager::clear()
{
  UniqueLock lock(instance().mutex_);

  // clear sub-databases
  mutableInstance().foothold_db_->clear();
  mutableInstance().floating_base_db_->clear();
  mutableInstance().state_db_->clear();
  mutableInstance().transition_db_->clear();

  // clear planning states
  mutableInstance().map_.clear();
  mutableInstance().uid_to_state_.clear();

  if (instance().state_ID2_index_mapping_)
  {
    for (int* entry : *mutableInstance().state_ID2_index_mapping_)
      delete[] entry;

    mutableInstance().state_ID2_index_mapping_->clear();

    // add dummy entry for id = 0
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    mutableInstance().state_ID2_index_mapping_->push_back(entry);

    for (int i = 0; i < NUMOFINDICES_STATEID2IND; i++)
      (*mutableInstance().state_ID2_index_mapping_)[0][i] = -1;
  }

  instance().hits_ = 0u;
  instance().miss_ = 0u;
}

FootholdHashed::ConstPtr StateSpaceManager::addFoothold(const Foothold& foothold)
{
  SharedLock lock(instance().mutex_);  // only shared lock in order to enable parallel use of all dbs (dbs are internally thread-safe)
  return mutableInstance().foothold_db_->insert(foothold);
}

FootholdHashedConstPtrArray StateSpaceManager::addFootholds(const FootholdConstPtrArray& footholds)
{
  FootholdHashedConstPtrArray result;
  for (Foothold::ConstPtr f : footholds)
    result.push_back(addFoothold(f));

  return result;
}

FootholdHashedConstPtrArray StateSpaceManager::addFootholds(const FootholdArray& footholds)
{
  FootholdHashedConstPtrArray result;
  for (const Foothold& f : footholds)
    result.push_back(addFoothold(f));

  return result;
}

FloatingBaseHashed::ConstPtr StateSpaceManager::addFloatingBase(const FloatingBase& floating_base)
{
  SharedLock lock(instance().mutex_); // only shared lock in order to enable parallel use of all dbs (dbs are internally thread-safe)
  return mutableInstance().floating_base_db_->insert(floating_base);
}

FloatingBaseHashedConstPtrArray StateSpaceManager::addFloatingBases(const FloatingBaseConstPtrArray& floating_bases)
{
  FloatingBaseHashedConstPtrArray result;
  for (const FloatingBase::ConstPtr& fb : floating_bases)
    result.push_back(addFloatingBase(fb));

  return result;
}

FloatingBaseHashedConstPtrArray StateSpaceManager::addFloatingBases(const FloatingBaseArray& floating_bases)
{
  FloatingBaseHashedConstPtrArray result;
  for (const FloatingBase& fb : floating_bases)
    result.push_back(addFloatingBase(fb));

  return result;
}

StateHashed::ConstPtr StateSpaceManager::addState(const State& state)
{
  SharedLock lock(instance().mutex_);  // only shared lock in order to enable parallel use of all dbs (dbs are internally thread-safe)
  return mutableInstance().state_db_->insert(state);
}

StateHashed::ConstPtr StateSpaceManager::addState(State&& state)
{
  SharedLock lock(instance().mutex_);  // only shared lock in order to enable parallel use of all dbs (dbs are internally thread-safe)
  return mutableInstance().state_db_->insert(std::move(state));
}

TransitionHashed::ConstPtr StateSpaceManager::addTransition(const Transition& transition)
{
  SharedLock lock(instance().mutex_);  // only shared lock in order to enable parallel use of all dbs (dbs are internally thread-safe)
  return mutableInstance().transition_db_->insert(transition);
}

TransitionHashed::ConstPtr StateSpaceManager::addTransition(Transition&& transition)
{
  SharedLock lock(instance().mutex_);  // only shared lock in order to enable parallel use of all dbs (dbs are internally thread-safe)
  return mutableInstance().transition_db_->insert(std::move(transition));
}

TransitionHashed::ConstPtr StateSpaceManager::getTransition(const StateID& pred, const StateID& succ)
{
  SharedLock lock(instance().mutex_);
  return instance().transition_db_->get(pred, succ);
}

TransitionHashed::ConstPtr StateSpaceManager::getTransition(const TransitionID& id)
{
  SharedLock lock(instance().mutex_);
  return instance().transition_db_->get(id);
}

TransitionSet StateSpaceManager::getPredecessors(const StateID& id)
{
  SharedLock lock(instance().mutex_);
  return instance().transition_db_->getPredecessors(id);
}

TransitionSet StateSpaceManager::getSuccessors(const StateID& id)
{
  SharedLock lock(instance().mutex_);
  return instance().transition_db_->getSuccessors(id);
}

PlanningStateHashed::ConstPtr StateSpaceManager::createPlanningState(const PlanningState& pstate)
{
  // check if entry does already exist
  PlanningStateID id(pstate);
  PlanningStateHashed::ConstPtr ptr = instance().getPlanningState(id);
  if (ptr)
    return ptr;

  // create new entry
  UniqueLock lock(instance().mutex_);
  ptr = PlanningStateHashed::make(pstate, id, instance().uid_to_state_.size() + 1);
  auto p = mutableInstance().map_.emplace(id, ptr);

  // check if state was really added; duplicated adds may happen as first try to get happend in shared lock environment
  if (p.second)
  {
    mutableInstance().uid_to_state_.push_back(ptr);

    // register at SBPL
    mutableInstance().createSbplPlanningEntry(*ptr);
  }

  return p.first->second;
}

PlanningStateHashed::ConstPtr StateSpaceManager::createPlanningState(PlanningState&& pstate)
{
  // check if entry does already exist
  PlanningStateID id(pstate);
  PlanningStateHashed::ConstPtr ptr = instance().getPlanningState(id);
  if (ptr)
    return ptr;

  // create new entry
  UniqueLock lock(instance().mutex_);
  // WARNING: Using move operation here; do not use pstate object from here
  ptr = PlanningStateHashed::make(std::move(pstate), id, instance().uid_to_state_.size() + 1);
  auto p = mutableInstance().map_.emplace(id, ptr);

  // check if state was really added; duplicated adds may happen as first try to get happend in shared lock environment
  if (p.second)
  {
    mutableInstance().uid_to_state_.push_back(ptr);

    // register at SBPL
    mutableInstance().createSbplPlanningEntry(*ptr);
  }

  return p.first->second;
}

PlanningStateHashed::ConstPtr StateSpaceManager::getPlanningState(const UID& uid)
{
  SharedLock lock(instance().mutex_);

  if (uid == 0 || instance().uid_to_state_.size() < uid)
  {
    instance().miss_++;
    return PlanningStateHashed::ConstPtr();
  }
  else
  {
    instance().hits_++;
    return mutableInstance().uid_to_state_[uid - 1];
  }
}

PlanningStateHashed::ConstPtr StateSpaceManager::getPlanningState(const PlanningStateID& id) const
{
  SharedLock lock(mutex_);

  PlanningStateMap::const_iterator itr = map_.find(id);
  if (itr != map_.end())
  {
    hits_++;
    return itr->second;
  }
  else
  {
    miss_++;
    return PlanningStateHashed::ConstPtr();
  }
}

void StateSpaceManager::createSbplPlanningEntry(const PlanningStateHashed& pstate)
{
  UID state_id = pstate.getUID();

  ROS_ASSERT(state_ID2_index_mapping_);
  ROS_ASSERT(state_id < static_cast<UID>(std::numeric_limits<int>::max()));

  int* entry = new int[NUMOFINDICES_STATEID2IND];
  state_ID2_index_mapping_->push_back(entry);

  for (int i = 0; i < NUMOFINDICES_STATEID2IND; i++)
    (*state_ID2_index_mapping_)[state_id][i] = -1;

  ROS_ASSERT(state_ID2_index_mapping_->size() - 1 == state_id);
}
}  // namespace l3_footstep_planning
