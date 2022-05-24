#include <l3_footstep_planning_libs/modeling/floating_base_database.h>

namespace l3_footstep_planning
{
FloatingBaseDataBase::FloatingBaseDataBase(const DiscreteResolution& resolution)
  : resolution_(resolution)
  , hits_(0u)
  , miss_(0u)
{}

size_t FloatingBaseDataBase::size() const
{
  SharedLock lock(mutex_);
  return map_.size();
}

bool FloatingBaseDataBase::empty() const
{
  SharedLock lock(mutex_);
  return map_.empty();
}

void FloatingBaseDataBase::clear()
{
  UniqueLock lock(mutex_);
  map_.clear();
  hits_ = 0u;
  miss_ = 0u;
}

FloatingBaseHashed::ConstPtr FloatingBaseDataBase::insert(const FloatingBase& floating_base)
{
  FloatingBaseID id(floating_base, resolution_);

  // check if entry does already exist
  FloatingBaseHashed::ConstPtr cptr = get(id);
  if (cptr)
    return cptr;

  // create new discretized entry
  FloatingBaseHashed::Ptr ptr = FloatingBaseHashed::make(id.getDiscreteFloatingBase(resolution_), id, map_.size() + 1);
  ptr->setRPY(ptr->roll(), ptr->pitch(), ptr->yaw());
  ptr->header = floating_base.header;
  ptr->data = floating_base.data;

  UniqueLock lock(mutex_);
  auto p = map_.emplace(id, ptr);

  // duplicated adds may happen as first try to get happend in shared lock environment
  return p.first->second;
}

FloatingBaseHashed::ConstPtr FloatingBaseDataBase::insert(FloatingBase::ConstPtr floating_base)
{
  ROS_ASSERT(floating_base);
  return insert(*floating_base);
}

bool FloatingBaseDataBase::has(const FloatingBaseID& id) const
{
  SharedLock lock(mutex_);
  return map_.find(id) != map_.end();
}

void FloatingBaseDataBase::remove(const FloatingBaseID& id)
{
  UniqueLock lock(mutex_);
  map_.erase(id);
}

FloatingBaseHashed::ConstPtr FloatingBaseDataBase::get(const FloatingBaseID& id) const
{
  SharedLock lock(mutex_);
  FloatingBaseMap::const_iterator itr = map_.find(id);
  if (itr != map_.end())
  {
    hits_++;
    return itr->second;
  }
  else
  {
    miss_++;
    return FloatingBaseHashed::ConstPtr();
  }
}
}  // namespace l3_footstep_planning
