#include <l3_footstep_planning_libs/modeling/transition_database.h>

namespace l3_footstep_planning
{
TransitionDataBase::TransitionDataBase()
  : hits_(0u)
  , miss_(0u)
{}

size_t TransitionDataBase::size() const
{
  SharedLock lock(mutex_);
  return map_.size();
}

bool TransitionDataBase::empty() const
{
  SharedLock lock(mutex_);
  return map_.empty();
}

void TransitionDataBase::clear()
{
  UniqueLock lock(mutex_);
  map_.clear();
  pred_map_.clear();
  succ_map_.clear();
  hits_ = 0u;
  miss_ = 0u;
}

TransitionHashed::ConstPtr TransitionDataBase::insert(const Transition& transition)
{
  ROS_ASSERT(transition.getPredecessor());
  ROS_ASSERT(transition.getSuccessor());

  // check if entry does already exist
  TransitionID id(transition.getPredecessor()->getID(), transition.getSuccessor()->getID());
  TransitionHashed::ConstPtr ptr = get(id);
  if (ptr)
    return ptr;

  // create new entry
  UniqueLock lock(mutex_);

  ptr = TransitionHashed::make(transition, id);
  auto p = map_.emplace(id, ptr);

  // check if state was really added; duplicated adds may happen as first try to get happend in shared lock environment
  if (p.second)
  {
    pred_map_[ptr->getSuccessor()->getID()].insert(ptr);
    succ_map_[ptr->getPredecessor()->getID()].insert(ptr);
  }

  return p.first->second;
}

TransitionHashed::ConstPtr TransitionDataBase::insert(Transition&& transition)
{
  ROS_ASSERT(transition.getPredecessor());
  ROS_ASSERT(transition.getSuccessor());

  // check if entry does already exist
  TransitionID id(transition.getPredecessor()->getID(), transition.getSuccessor()->getID());
  TransitionHashed::ConstPtr ptr = get(id);
  if (ptr)
    return ptr;

  // create new entry
  UniqueLock lock(mutex_);

  // WARNING: Using move operation here; do not use transition object from here
  ptr = TransitionHashed::make(std::move(transition), id);
  auto p = map_.emplace(id, ptr);

  // check if state was really added; duplicated adds may happen as first try to get happend in shared lock environment
  if (p.second)
  {
    pred_map_[ptr->getSuccessor()->getID()].insert(ptr);
    succ_map_[ptr->getPredecessor()->getID()].insert(ptr);
  }

  return p.first->second;
}

bool TransitionDataBase::has(const TransitionID& id) const
{
  SharedLock lock(mutex_);
  return map_.find(id) != map_.end();
}

TransitionHashed::ConstPtr TransitionDataBase::get(const TransitionID& id) const
{
  SharedLock lock(mutex_);
  TransitionMap::const_iterator itr = map_.find(id);
  if (itr != map_.end())
  {
    hits_++;
    return itr->second;
  }
  else
  {
    miss_++;
    return TransitionHashed::ConstPtr();
  }
}

const TransitionSet& TransitionDataBase::getPredecessors(const StateID& id) const
{
  SharedLock lock(mutex_);
  TransitionListMap::const_iterator itr = pred_map_.find(id);
  if (itr != pred_map_.end())
  {
    hits_++;
    return itr->second;
  }
  else
  {
    miss_++;
    return empty_set_;
  }
}

const TransitionSet& TransitionDataBase::getSuccessors(const StateID& id) const
{
  SharedLock lock(mutex_);
  TransitionListMap::const_iterator itr = succ_map_.find(id);
  if (itr != succ_map_.end())
  {
    hits_++;
    return itr->second;
  }
  else
  {
    miss_++;
    return empty_set_;
  }
}

void TransitionDataBase::remove(const TransitionID& id)
{
  UniqueLock lock(mutex_);

  TransitionMap::const_iterator itr = map_.find(id);
  if (itr == map_.end())
    return;

  pred_map_[id.getSuccessorID()].erase(itr->second);
  succ_map_[id.getPredecessorID()].erase(itr->second);
  map_.erase(itr);
}
}  // namespace l3_footstep_planning
