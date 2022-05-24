#include <l3_footstep_planning_libs/modeling/foothold_database.h>

namespace l3_footstep_planning
{
FootholdDataBase::FootholdDataBase(const DiscreteResolution& resolution)
  : resolution_(resolution)
  , hits_(0u)
  , miss_(0u)
{}

size_t FootholdDataBase::size() const
{
  SharedLock lock(mutex_);
  return map_.size();
}

bool FootholdDataBase::empty() const
{
  SharedLock lock(mutex_);
  return map_.empty();
}

void FootholdDataBase::clear()
{
  UniqueLock lock(mutex_);
  map_.clear();
  hits_ = 0u;
  miss_ = 0u;
}

FootholdHashed::ConstPtr FootholdDataBase::insert(const Foothold& foothold)
{
  FootholdID id(foothold, resolution_);

  // check if entry does already exist
  FootholdHashed::ConstPtr cptr = get(id);
  if (cptr)
    return cptr;

  // create new discretized entry
  FootholdHashed::Ptr ptr = FootholdHashed::make(id.getDiscreteFoothold(resolution_), id, map_.size() + 1);
  ptr->setRPY(foothold.roll(), foothold.pitch(), ptr->yaw());
  ptr->header = foothold.header;
  ptr->data = foothold.data;

  UniqueLock lock(mutex_);
  auto p = map_.emplace(id, ptr);

  // duplicated adds may happen as first try to get happend in shared lock environment
  return p.first->second;
}

FootholdHashed::ConstPtr FootholdDataBase::insert(Foothold::ConstPtr foothold)
{
  ROS_ASSERT(foothold);
  return insert(*foothold);
}

bool FootholdDataBase::has(const FootholdID& id) const
{
  SharedLock lock(mutex_);
  return map_.find(id) != map_.end();
}

void FootholdDataBase::remove(const FootholdID& id)
{
  UniqueLock lock(mutex_);
  map_.erase(id);
}

FootholdHashed::ConstPtr FootholdDataBase::get(const FootholdID& id) const
{
  SharedLock lock(mutex_);
  FootholdMap::const_iterator itr = map_.find(id);
  if (itr != map_.end())
  {
    hits_++;
    return itr->second;
  }
  else
  {
    miss_++;
    return FootholdHashed::ConstPtr();
  }
}
}  // namespace l3_footstep_planning
