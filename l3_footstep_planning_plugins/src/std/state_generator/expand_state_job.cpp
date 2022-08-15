#include <l3_footstep_planning_plugins/std/state_generator/expand_state_job.h>

#include <l3_footstep_planning_plugins/aggregator/reachability.h>
#include <l3_footstep_planning_plugins/aggregator/world_model.h>
#include <l3_footstep_planning_plugins/aggregator/step_cost_estimator.h>
#include <l3_footstep_planning_plugins/aggregator/post_processor.h>

#include <l3_footstep_planning_libs/modeling/state_space_manager.h>

namespace l3_footstep_planning
{
namespace threading
{
ExpandStateJob::ExpandStateJob(PlanningState::ConstPtr state, State::ConstPtr start, State::ConstPtr goal, const FootholdPtrArray& new_footholds,
                               FloatingBase::ConstPtr new_floating_base, bool forward, bool lazy, double basic_cost, double basic_risk)
  : cost(basic_cost)
  , risk(basic_risk)
  , successful(false)
  , current_state_(state->getState())
  , start_(start)
  , goal_(goal)
  , new_footholds_(new_footholds)
  , new_floating_base_(new_floating_base)
  , forward_(forward)
  , lazy_(lazy)
{}

ExpandStateJob::ExpandStateJob(PlanningState::ConstPtr state, State::ConstPtr start, State::ConstPtr goal, const FootholdHashedConstPtrArray& new_footholds,
                               FloatingBaseHashed::ConstPtr new_floating_base, bool forward, bool lazy, double basic_cost, double basic_risk)
  : cost(basic_cost)
  , risk(basic_risk)
  , successful(false)
  , current_state_(state->getState())
  , start_(start)
  , goal_(goal)
  , new_footholds_hashed_(new_footholds)
  , new_floating_base_hashed_(new_floating_base)
  , forward_(forward)
  , lazy_(lazy)
{}

void ExpandStateJob::run()
{
  if (new_footholds_.empty() && new_footholds_hashed_.empty() && !new_floating_base_ && !new_floating_base_hashed_)
    return;

  successful = false;

  /// check footholds
  FootIndexArray updated_ids;

  // create hashed footholds
  if (new_footholds_hashed_.empty())
  {
    for (Foothold::Ptr f : new_footholds_)
    {
      /// lookup for cached footholds
      FootholdHashed::ConstPtr f_ptr = StateSpaceManager::getFoothold(FootholdKey(f));

      // otherwise generate new 6 DoF foothold object
      if (!f_ptr)
      {
        // update 3D pose based on world data
        if (WorldModel::instance().update3DData(*f) < TerrainResult::OK)
          return;

        bool success = PostProcessor::instance().postProcess(*f);

        f_ptr = StateSpaceManager::addFoothold(f);

        if (!success)
          return;
      }

      if (!WorldModel::instance().isAccessible(*f_ptr))
        return;

      ROS_ASSERT(f_ptr);
      ROS_ASSERT(f->idx == f_ptr->idx);

      new_footholds_hashed_.push_back(f_ptr);
      updated_ids.push_back(f_ptr->idx);
    }
  }
  else
  {
    for (Foothold::ConstPtr f : new_footholds_hashed_)
      updated_ids.push_back(f->idx);
  }

  /// check floating base
  /// @todo extend to multiple bases planning
  if (!new_floating_base_hashed_ && new_floating_base_)
  {
    FloatingBase floating_base = *new_floating_base_;

    if (!PostProcessor::instance().postProcess(floating_base))
      return;

    if (!WorldModel::instance().isAccessible(floating_base))
      return;

    new_floating_base_hashed_ = StateSpaceManager::addFloatingBase(floating_base);
  }

  /// generate new state
  State new_state;

  // copy footholds from current state
  new_state.updateFootholds(current_state_->getFootholdsHashed());

  // updated footholds
  new_state.updateFootholds(new_footholds_hashed_);

  // copy floating bases from current state
  new_state.updateFloatingBases(current_state_->getFloatingBasesHashed());

  // update floating base
  if (new_floating_base_hashed_)
    new_state.updateFloatingBase(new_floating_base_hashed_);

  /// perform state caching
  StateHashed::ConstPtr new_hashed_state = StateSpaceManager::getState(new_state.getFootholdsHashed(), new_state.getFloatingBasesHashed());

  // post process newly generated state if state space manager has not returned a cached state
  if (!new_hashed_state)
  {
    /// @todo updated ids should be expanded ids
    bool success = PostProcessor::instance().postProcess(new_state, updated_ids);
    new_state.setValid(success);

    // WARNING: Using move operation here; do not use new_state object from here
    new_hashed_state = StateSpaceManager::addState(std::move(new_state));
  }

  if (!new_hashed_state->isValid())
    return;

  /// perform planning state caching
  FootIndexSet moved_feet_idx = current_state_->getChangedFootholdIdx(*new_hashed_state);
  BaseIndexSet moved_base_idx = current_state_->getChangedFloatingBaseIdx(*new_hashed_state);

  if (forward_)
    next = StateSpaceManager::getPlanningState(new_hashed_state, current_state_, StateHashed::ConstPtr(), moved_feet_idx, moved_base_idx);
  else
    next = StateSpaceManager::getPlanningState(new_hashed_state, StateHashed::ConstPtr(), current_state_, moved_feet_idx, moved_base_idx);

  if (!next)
  {
    // clang-format off
    // create planning state
    PlanningState new_planning_state(new_hashed_state, forward_ ? current_state_ : StateHashed::ConstPtr(), forward_ ? StateHashed::ConstPtr() : current_state_);
    // clang-format on

    // apply post processing steps
    bool success = PostProcessor::instance().postProcess(new_planning_state);
    if (success)
      success &= PostProcessor::instance().postProcess(*new_planning_state.getStep());

    // check reachability due to discretization
    if (success)
      success &= Reachability::instance().isReachable(new_planning_state);

    // collision check
    if (success)
      success &= WorldModel::instance().isAccessible(new_planning_state);

    new_planning_state.setValid(success);

    // WARNING: Using move operation here; do not use new_planning_state object from here
    next = StateSpaceManager::createPlanningState(std::move(new_planning_state));
  }

  if (!next->isValid())
    return;

  // lookup costs
  if (!lazy_)
  {
    double c, r;
    if (!StepCostEstimator::instance().getCost(*next, *start_, *goal_, c, r))
      return;

    cost += c;
    risk += r;
  }
  // add transition
  //  Transition t(Footstep::ConstPtr(), state_, next, cost, risk);
  //  StateSpaceManager::addTransition(t);

  successful = true;
}
}  // namespace threading
}  // namespace l3_footstep_planning
