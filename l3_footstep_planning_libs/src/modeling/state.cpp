#include <l3_footstep_planning_libs/modeling/state.h>

#include <l3_libs/helper.h>
#include <l3_plugins/robot_model.h>

namespace l3_footstep_planning
{
State::State()
  : feet_center_dirty_(true)
  , is_start_(false)
  , is_goal_(false)
  , valid_(true)
{}

State::State(const FootholdHashedConstPtrArray& footholds, const FloatingBaseHashedConstPtrArray& floating_bases)
  : is_start_(false)
  , is_goal_(false)
  , valid_(true)
{
  updateFootholds(footholds);
  updateFloatingBases(floating_bases);
}

State::State(const State& other, SharedLock /*other_lock*/)
  : foothold_map_(other.foothold_map_)
  , footholds_hashed_(other.footholds_hashed_)
  , footholds_(other.footholds_)
  , floating_base_map_(other.floating_base_map_)
  , floating_bases_hashed_(other.floating_bases_hashed_)
  , floating_bases_(other.floating_bases_)
  , feet_center_dirty_(other.feet_center_dirty_)
  , feet_center_(other.feet_center_)
  , is_start_(other.is_start_)
  , is_goal_(other.is_goal_)
  , valid_(other.valid_)
{}

State::State(State&& other, UniqueLock /*other_lock*/)
  : foothold_map_(std::move(other.foothold_map_))
  , footholds_hashed_(std::move(other.footholds_hashed_))
  , footholds_(std::move(other.footholds_))
  , floating_base_map_(std::move(other.floating_base_map_))
  , floating_bases_hashed_(std::move(other.floating_bases_hashed_))
  , floating_bases_(std::move(other.floating_bases_))
  , feet_center_dirty_(std::move(other.feet_center_dirty_))
  , feet_center_(std::move(other.feet_center_))
  , is_start_(std::move(other.is_start_))
  , is_goal_(std::move(other.is_goal_))
  , valid_(other.valid_)
{}

State& State::swap(State& other)
{
  if (*this != other)
  {
    UniqueLock this_lock(mutex_, boost::defer_lock);
    UniqueLock other_lock(other.mutex_, boost::defer_lock);
    std::lock(this_lock, other_lock);

    foothold_map_.swap(other.foothold_map_);
    footholds_hashed_.swap(other.footholds_hashed_);
    footholds_.swap(other.footholds_);
    floating_base_map_.swap(other.floating_base_map_);
    floating_bases_hashed_.swap(other.floating_bases_hashed_);
    floating_bases_.swap(other.floating_bases_);
    std::swap(feet_center_dirty_, other.feet_center_dirty_);
    std::swap(feet_center_, other.feet_center_);
    std::swap(is_start_, other.is_start_);
    std::swap(is_goal_, other.is_goal_);
    std::swap(valid_, other.valid_);
  }
  return *this;
}

State& State::operator=(const State& other)
{
  if (*this != other)
  {
    UniqueLock this_lock(mutex_, boost::defer_lock);
    SharedLock other_lock(other.mutex_, boost::defer_lock);
    std::lock(this_lock, other_lock);

    foothold_map_ = other.foothold_map_;
    footholds_hashed_ = other.footholds_hashed_;
    footholds_ = other.footholds_;
    floating_base_map_ = other.floating_base_map_;
    floating_bases_hashed_ = other.floating_bases_hashed_;
    floating_bases_ = other.floating_bases_;
    feet_center_dirty_ = other.feet_center_dirty_;
    feet_center_ = other.feet_center_;
    is_start_ = other.is_start_;
    is_goal_ = other.is_goal_;
    valid_ = other.valid_;
  }
  return *this;
}

State& State::operator=(State&& other)
{
  if (*this != other)
  {
    UniqueLock this_lock(mutex_, boost::defer_lock);
    UniqueLock other_lock(other.mutex_, boost::defer_lock);
    std::lock(this_lock, other_lock);

    foothold_map_ = std::move(other.foothold_map_);
    footholds_hashed_ = std::move(other.footholds_hashed_);
    footholds_ = std::move(other.footholds_);
    floating_base_map_ = std::move(other.floating_base_map_);
    floating_bases_hashed_ = std::move(other.floating_bases_hashed_);
    floating_bases_ = std::move(other.floating_bases_);
    feet_center_dirty_ = std::move(other.feet_center_dirty_);
    feet_center_ = std::move(other.feet_center_);
    is_start_ = std::move(other.is_start_);
    is_goal_ = std::move(other.is_goal_);
    valid_ = std::move(other.valid_);
  }
  return *this;
}

bool State::operator==(const State& other) const
{
  if (valid_ != other.valid_)
    return false;

  // check if floating bases are matching
  if (floating_base_map_.size() != other.floating_base_map_.size())
    return false;

  for (const std::pair<BaseIndex, FloatingBaseHashed::ConstPtr>& p : floating_base_map_)
  {
    State::FloatingBaseMap::const_iterator itr = other.floating_base_map_.find(p.first);
    if (itr == other.floating_base_map_.end())
      return false;

    if (p.second->getUID() != itr->second->getUID())
      return false;
  }

  // check if footholds are matching
  if (foothold_map_.size() != other.foothold_map_.size())
    return false;

  for (const std::pair<FootIndex, FootholdHashed::ConstPtr>& p : foothold_map_)
  {
    State::FootholdMap::const_iterator itr = other.foothold_map_.find(p.first);
    if (itr == other.foothold_map_.end())
      return false;

    if (p.second->getUID() != itr->second->getUID())
      return false;
  }

  return true;
}

bool State::operator!=(const State& other) const { return not(*this == other); }

void State::clear()
{
  clearFootholds();
  clearFloatingBases();

  is_start_ = false;
  is_goal_ = false;
}

void State::clearFootholds()
{
  UniqueLock lock(mutex_);

  foothold_map_.clear();
  footholds_hashed_.clear();
  footholds_.clear();

  feet_center_dirty_ = true;
}

bool State::hasFootholds() const
{
  SharedLock lock(mutex_);
  return !foothold_map_.empty();
}

void State::updateFoothold(FootholdHashed::ConstPtr foothold)
{
  UniqueLock lock(mutex_);

  foothold_map_[foothold->idx] = foothold;

  // rebuild lists
  rebuildFeetLists();

  feet_center_dirty_ = true;
}

void State::updateFootholds(FootholdHashedConstPtrArray footholds)
{
  if (footholds.empty())
    return;

  UniqueLock lock(mutex_);

  for (FootholdHashed::ConstPtr foothold : footholds)
    foothold_map_[foothold->idx] = foothold;

  // rebuild lists
  rebuildFeetLists();

  feet_center_dirty_ = true;
}

FootholdHashed::ConstPtr State::getFoothold(const FootIndex& foot_idx) const
{
  SharedLock lock(mutex_);

  State::FootholdMap::const_iterator itr = foothold_map_.find(foot_idx);

  if (itr == foothold_map_.end())
    return FootholdHashed::ConstPtr();

  return itr->second;
}

FootholdHashedConstPtrArray State::getChangedFootholds(const State& other) const
{
  FootholdHashedConstPtrArray footholds;

  for (const std::pair<FootIndex, FootholdHashed::ConstPtr>& p : foothold_map_)
  {
    ROS_ASSERT(p.second);

    FootholdHashed::ConstPtr f = other.getFoothold(p.first);

    if (f && f->getUID() != p.second->getUID())
      footholds.push_back(f);
  }

  return footholds;
}

FootIndexSet State::getChangedFootholdIdx(const State& other) const
{
  FootIndexSet idx;

  for (const std::pair<FootIndex, FootholdHashed::ConstPtr>& p : foothold_map_)
  {
    ROS_ASSERT(p.second);

    FootholdHashed::ConstPtr f = other.getFoothold(p.first);

    if (f && f->getUID() != p.second->getUID())
      idx.insert(f->idx);
  }

  return idx;
}

FootholdHashedConstPtrArray State::getUnchangedFootholds(const State& other) const
{
  FootholdHashedConstPtrArray footholds;

  for (const std::pair<FootIndex, FootholdHashed::ConstPtr>& p : foothold_map_)
  {
    ROS_ASSERT(p.second);

    FootholdHashed::ConstPtr f = other.getFoothold(p.first);

    if (f && f->getUID() == p.second->getUID())
      footholds.push_back(f);
  }

  return footholds;
}

FootIndexSet State::getUnchangedFootholdIdx(const State& other) const
{
  FootIndexSet idx;

  for (const std::pair<FootIndex, FootholdHashed::ConstPtr>& p : foothold_map_)
  {
    ROS_ASSERT(p.second);

    FootholdHashed::ConstPtr f = other.getFoothold(p.first);

    if (f && f->getUID() == p.second->getUID())
      idx.insert(f->idx);
  }

  return idx;
}

void State::clearFloatingBases()
{
  UniqueLock lock(mutex_);
  floating_base_map_.clear();
  floating_bases_hashed_.clear();
  floating_bases_.clear();
}

bool State::hasFloatingBases() const
{
  SharedLock lock(mutex_);
  return !floating_base_map_.empty();
}

void State::updateFloatingBase(FloatingBaseHashed::ConstPtr floating_base)
{
  UniqueLock lock(mutex_);

  floating_base_map_[floating_base->idx] = floating_base;

  rebuildFloatingBasesLists();
}

void State::updateFloatingBases(FloatingBaseHashedConstPtrArray floating_bases)
{
  if (floating_bases.empty())
    return;

  UniqueLock lock(mutex_);

  for (FloatingBaseHashed::ConstPtr floating_base : floating_bases)
    floating_base_map_[floating_base->idx] = floating_base;

  rebuildFloatingBasesLists();
}

FloatingBaseHashed::ConstPtr State::getFloatingBase(const BaseIndex& base_idx) const
{
  SharedLock lock(mutex_);

  State::FloatingBaseMap::const_iterator itr = floating_base_map_.find(base_idx);

  if (itr == floating_base_map_.end())
    return FloatingBaseHashed::ConstPtr();

  return itr->second;
}

FloatingBaseHashedConstPtrArray State::getChangedFloatingBases(const State& other) const
{
  FloatingBaseHashedConstPtrArray floating_bases;

  for (const std::pair<BaseIndex, FloatingBaseHashed::ConstPtr>& p : floating_base_map_)
  {
    ROS_ASSERT(p.second);

    FloatingBaseHashed::ConstPtr fb = other.getFloatingBase(p.first);

    if (fb && fb->getUID() != p.second->getUID())
      floating_bases.push_back(fb);
  }

  return floating_bases;
}

BaseIndexSet State::getChangedFloatingBaseIdx(const State& other) const
{
  BaseIndexSet idx;

  for (const std::pair<BaseIndex, FloatingBaseHashed::ConstPtr>& p : floating_base_map_)
  {
    ROS_ASSERT(p.second);

    FloatingBaseHashed::ConstPtr fb = other.getFloatingBase(p.first);

    if (fb && fb->getUID() != p.second->getUID())
      idx.insert(fb->idx);
  }

  return idx;
}

FloatingBaseHashedConstPtrArray State::getUnchangedFloatingBases(const State& other) const
{
  FloatingBaseHashedConstPtrArray floating_bases;

  for (const std::pair<BaseIndex, FloatingBaseHashed::ConstPtr>& p : floating_base_map_)
  {
    ROS_ASSERT(p.second);

    FloatingBaseHashed::ConstPtr fb = other.getFloatingBase(p.first);

    if (fb && fb->getUID() == p.second->getUID())
      floating_bases.push_back(fb);
  }

  return floating_bases;
}

BaseIndexSet State::getUnchangedFloatingBaseIdx(const State& other) const
{
  BaseIndexSet idx;

  for (const std::pair<BaseIndex, FloatingBaseHashed::ConstPtr>& p : floating_base_map_)
  {
    ROS_ASSERT(p.second);

    FloatingBaseHashed::ConstPtr fb = other.getFloatingBase(p.first);

    if (fb && fb->getUID() == p.second->getUID())
      idx.insert(fb->idx);
  }

  return idx;
}

const Pose& State::getFeetCenter() const
{
  UpgradeLock lock(mutex_);

  if (feet_center_dirty_)
  {
    UpgradeToUniqueLock uniqueLock(lock);
    feet_center_ = RobotModel::calcFeetCenter(getFootholds());
    feet_center_dirty_ = false;
  }

  return feet_center_;
}

void State::rebuildFeetLists()
{
  footholds_hashed_.clear();
  footholds_.clear();

  for (const FootholdPair& p : foothold_map_)
  {
    footholds_hashed_.push_back(p.second);
    footholds_.push_back(p.second);
  }
}

void State::rebuildFloatingBasesLists()
{
  floating_bases_hashed_.clear();
  floating_bases_.clear();

  for (const FloatingBasePair& p : floating_base_map_)
  {
    floating_bases_hashed_.push_back(p.second);
    floating_bases_.push_back(p.second);
  }
}
}  // namespace l3_footstep_planning
