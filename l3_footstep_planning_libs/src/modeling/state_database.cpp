#include <l3_footstep_planning_libs/modeling/state_database.h>

namespace l3_footstep_planning
{
StateDataBase::StateDataBase(const DiscreteResolution& resolution)
  : resolution_(resolution)
  , hits_(0u)
  , miss_(0u)
{}

size_t StateDataBase::size() const
{
  SharedLock lock(mutex_);
  return map_.size();
}

bool StateDataBase::empty() const
{
  SharedLock lock(mutex_);
  return map_.empty();
}

void StateDataBase::clear()
{
  UniqueLock lock(mutex_);
  map_.clear();
  uid_to_state_.clear();
  hits_ = 0u;
  miss_ = 0u;
}

StateHashed::ConstPtr StateDataBase::insert(const State& state)
{
  // check if entry does already exist
  StateID id(state);
  StateHashed::ConstPtr ptr = get(id);
  if (ptr)
    return ptr;

  // create new entry
  UniqueLock lock(mutex_);
  ptr = StateHashed::make(state, id, uid_to_state_.size() + 1);
  auto p = map_.emplace(id, ptr);

  // check if state was really added; duplicated adds may happen as first try to get happend in shared lock environment
  if (p.second)
    uid_to_state_.push_back(ptr);

  return p.first->second;
}

StateHashed::ConstPtr StateDataBase::insert(State&& state)
{
  // check if entry does already exist
  StateID id(state);
  StateHashed::ConstPtr ptr = get(id);
  if (ptr)
    return ptr;

  // create new entry
  UniqueLock lock(mutex_);
  // WARNING: Using move operation here; do not use state object from here
  ptr = StateHashed::make(std::move(state), id, uid_to_state_.size() + 1);
  auto p = map_.emplace(id, ptr);

  // check if state was really added; duplicated adds may happen as first try to get happend in shared lock environment
  if (p.second)
    uid_to_state_.push_back(ptr);

  return p.first->second;
}

StateHashed::ConstPtr StateDataBase::insert(State::ConstPtr state)
{
  ROS_ASSERT(state);
  return insert(*state);
}

bool StateDataBase::has(const UID& uid) const
{
  SharedLock lock(mutex_);

  if (uid == 0 || uid_to_state_.size() < uid || !uid_to_state_[uid - 1])
    return false;

  return true;
}

bool StateDataBase::has(const StateID& id) const
{
  SharedLock lock(mutex_);
  return map_.find(id) != map_.end();
}

StateHashed::ConstPtr StateDataBase::get(const UID& uid) const
{
  SharedLock lock(mutex_);

  if (uid == 0 || uid_to_state_.size() < uid)
  {
    miss_++;
    return StateHashed::ConstPtr();
  }
  else
  {
    hits_++;
    return uid_to_state_[uid - 1];
  }
}

StateHashed::ConstPtr StateDataBase::get(const StateID& id) const
{
  SharedLock lock(mutex_);

  StateMap::const_iterator itr = map_.find(id);
  if (itr != map_.end())
  {
    hits_++;
    return itr->second;
  }
  else
  {
    miss_++;
    return StateHashed::ConstPtr();
  }
}

void StateDataBase::remove(const StateID& id)
{
  UniqueLock lock(mutex_);

  StateMap::const_iterator itr = map_.find(id);
  if (itr == map_.end())
    return;

  // Acceptable memory leak here in order to improve overall performance; However, deleting states is not intended during planning.
  uid_to_state_[itr->second->getUID() - 1] = StateHashed::ConstPtr();
  map_.erase(itr);
}
}  // namespace l3_footstep_planning
