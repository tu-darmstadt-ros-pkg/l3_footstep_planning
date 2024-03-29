#include <l3_footstep_planning_libs/modeling/planning_state.h>

namespace l3_footstep_planning
{
PlanningState::PlanningState(StateHashed::ConstPtr state, StateHashed::ConstPtr pred, StateHashed::ConstPtr succ, Step::ConstPtr step)
  : state_(state)
  , pred_(pred)
  , succ_(succ)
  , valid_(true)
{
  ROS_ASSERT(state_);

  if (step)
    step_ = makeShared<Step>(*step);
  else
    step_ = makeShared<Step>();

  if (pred_)
    adj_ = pred_;

  if (succ_)
    adj_ = succ_;

  if (pred_ && succ_)
  {
    ROS_WARN_ONCE("PlanningState with predecessor AND successor was created. This violates the planning state convention and may causes unexpected behavior.");
    return;
  }

  // generate new step map if not explicitly given
  if (!step && adj_)
  {
    // generate foothold data
    for (FootholdHashed::ConstPtr adj_foothold : adj_->getFootholdsHashed())
    {
      ROS_ASSERT(adj_foothold);

      FootholdHashed::ConstPtr foothold = state_->getFoothold(adj_foothold->idx);

      if (!foothold)
        continue;

      // check if foot is moving
      if (foothold->getUID() != adj_foothold->getUID())
      {
        if (pred_)
          step_->footStep().updateMovingLink(foothold->idx, FootStepData::make(adj_foothold, foothold));
        if (succ_)
          step_->footStep().updateMovingLink(foothold->idx, FootStepData::make(foothold, adj_foothold));
      }
      // otherwise foothold is a support foot
      else
        step_->footStep().updateNonMovingLink(foothold->idx, foothold);
    }

    // generate floating base data
    for (FloatingBaseHashed::ConstPtr adj_floating_base : adj_->getFloatingBasesHashed())
    {
      ROS_ASSERT(adj_floating_base);

      FloatingBaseHashed::ConstPtr floating_base = state_->getFloatingBase(adj_floating_base->idx);

      if (!floating_base)
        continue;

      if (adj_floating_base->getUID() != floating_base->getUID())
      {
        // check if base is moving
        if (pred_)
          step_->baseStep().updateMovingLink(floating_base->idx, BaseStepData::make(adj_floating_base, floating_base));
        if (succ_)
          step_->baseStep().updateMovingLink(floating_base->idx, BaseStepData::make(floating_base, adj_floating_base));
      }
      // otherwise floating base is a support base
      else
        step_->baseStep().updateNonMovingLink(floating_base->idx, floating_base);
    }
  }
}

FootholdHashedConstPtrArray PlanningState::getChangedFootholds() const
{
  if (step_->footStep().hasMovingLinks())
  {
    FootholdHashedConstPtrArray footholds;
    for (const Step::FootStep::MovingDataPair& p : step_->footStep().getMovingLinks())
    {
      ROS_ASSERT(state_->getFoothold(p.first));
      footholds.push_back(state_->getFoothold(p.first));
    }
    return footholds;
  }
  else if (pred_)
    return pred_->getChangedFootholds(*state_);
  else if (succ_)
    return succ_->getChangedFootholds(*state_);

  return FootholdHashedConstPtrArray();
}

FootholdHashedConstPtrArray PlanningState::getUnchangedFootholds() const
{
  if (step_->footStep().hasNonMovingLinks())
  {
    FootholdHashedConstPtrArray footholds;
    for (const Step::FootStep::NonMovingDataPair& p : step_->footStep().getNonMovingLinks())
    {
      ROS_ASSERT(state_->getFoothold(p.first));
      footholds.push_back(state_->getFoothold(p.first));
    }
    return footholds;
  }
  else if (pred_)
    return pred_->getUnchangedFootholds(*state_);
  else if (succ_)
    return succ_->getUnchangedFootholds(*state_);

  return FootholdHashedConstPtrArray();
}

FloatingBaseHashedConstPtrArray PlanningState::getChangedFloatingBases() const
{
  if (step_->baseStep().hasMovingLinks())
  {
    FloatingBaseHashedConstPtrArray floating_bases;
    for (const Step::BaseStep::MovingDataPair& p : step_->baseStep().getMovingLinks())
    {
      ROS_ASSERT(state_->getFloatingBase(p.first));
      floating_bases.push_back(state_->getFloatingBase(p.first));
    }
    return floating_bases;
  }
  else if (pred_)
    return pred_->getChangedFloatingBases(*state_);
  else if (succ_)
    return succ_->getChangedFloatingBases(*state_);

  return FloatingBaseHashedConstPtrArray();
}

FloatingBaseHashedConstPtrArray PlanningState::getUnchangedFloatingBases() const
{
  if (step_->baseStep().hasNonMovingLinks())
  {
    FloatingBaseHashedConstPtrArray floating_bases;
    for (const Step::BaseStep::NonMovingDataPair& p : step_->baseStep().getNonMovingLinks())
    {
      ROS_ASSERT(state_->getFloatingBase(p.first));
      floating_bases.push_back(state_->getFloatingBase(p.first));
    }
    return floating_bases;
  }
  else if (pred_)
    return pred_->getUnchangedFloatingBases(*state_);
  else if (succ_)
    return succ_->getUnchangedFloatingBases(*state_);

  return FloatingBaseHashedConstPtrArray();
}


PlanningStateID::PlanningStateID(StateHashed::ConstPtr state, StateHashed::ConstPtr pred, StateHashed::ConstPtr succ, Step::ConstPtr step)
  : state_id_(state->getID())
  , pred_id_(pred ? pred->getID() : StateID())
  , succ_id_(succ ? succ->getID() : StateID())
{
  ROS_ASSERT(state);

  // compute hash value
  hash_ = 0;

  boost::hash_combine(hash_, state->getHashValue());

  if (pred)
    boost::hash_combine(hash_, pred->getHashValue());
  else if (succ)
    boost::hash_combine(hash_, succ->getHashValue());

  if (step)
  {
    for (const Step::FootStep::MovingDataPair& p : step->footStep().getMovingLinks())
    {
      foot_idx_.insert(p.first);
      boost::hash_combine(hash_, p.first);
    }

    for (const Step::BaseStep::MovingDataPair& p : step->baseStep().getMovingLinks())
    {
      base_idx_.insert(p.first);
      boost::hash_combine(hash_, p.first);
    }
  }
}

PlanningStateID::PlanningStateID(StateHashed::ConstPtr state, StateHashed::ConstPtr pred, StateHashed::ConstPtr succ, const FootIndexSet& moved_feet_idx, const BaseIndexSet& moved_bases_idx)
{
  ROS_ASSERT(state);

  // compute hash value
  hash_ = 0;

  boost::hash_combine(hash_, state->getHashValue());

  if (pred)
    boost::hash_combine(hash_, pred->getHashValue());
  else if (succ)
    boost::hash_combine(hash_, succ->getHashValue());
  
  foot_idx_ = moved_feet_idx;
  base_idx_ = moved_bases_idx;
  
  for (const FootIndex& idx : moved_feet_idx)
    boost::hash_combine(hash_, idx);

  for (const BaseIndex& idx : moved_bases_idx)
    boost::hash_combine(hash_, idx);
}

PlanningStateID::PlanningStateID(const PlanningState& state)
  : PlanningStateID(state.getState(), state.getPredState(), state.getSuccState(), state.getStep())
{}
}  // namespace l3_footstep_planning
