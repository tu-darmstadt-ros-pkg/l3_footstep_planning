#include <l3_footstep_planner/state_space/state_space.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_libs/math.h>

#include <l3_footstep_planning_plugins/aggregator/reachability.h>
#include <l3_footstep_planning_plugins/aggregator/post_processor.h>
#include <l3_footstep_planning_plugins/aggregator/step_cost_estimator.h>
#include <l3_footstep_planning_plugins/aggregator/heuristic.h>

namespace l3_footstep_planning
{
StateSpace::StateSpace(const EnvironmentParameters& params)
  : params_(params)
{}

void StateSpace::reset()
{
  start_state_.reset();
  goal_state_.reset();
}

UID StateSpace::updateStart(const State& state)
{
  State temp_state = state;

  // a plan request clears state database and all stored footholds, thus need to readd them first
  temp_state.updateFootholds(StateSpaceManager::addFootholds(state.getFootholds()));

  // WARNING: Using move operation here; do not use temp_state object from here
  StateHashed::ConstPtr state_ptr = StateSpaceManager::addState(std::move(temp_state));
  if (params_.forward_search)
    start_state_ = StateSpaceManager::createPlanningState(state_ptr, state_ptr);
  else
    start_state_ = StateSpaceManager::createPlanningState(state_ptr);
  return start_state_->getUID();
}

UID StateSpace::updateGoal(const State& state)
{
  State temp_state = state;

  // a plan request clears state database and all stored footholds, thus need to readd them first
  temp_state.updateFootholds(StateSpaceManager::addFootholds(state.getFootholds()));

  // WARNING: Using move operation here; do not use temp_state object from here
  StateHashed::ConstPtr state_ptr = StateSpaceManager::addState(std::move(temp_state));
  if (params_.forward_search)
    goal_state_ = StateSpaceManager::createPlanningState(state_ptr);
  else
    goal_state_ = StateSpaceManager::createPlanningState(state_ptr, StateHashed::ConstPtr(), state_ptr);

  // check reachability
  if (!Reachability::instance().isReachable(*goal_state_))
    ROS_WARN("[StateSpace] updateGoal: Invalid goal! Goal state failed reachability check between feet poses.");

  return goal_state_->getUID();
}
}  // namespace l3_footstep_planning
