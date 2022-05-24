#include <l3_footstep_planner/footstep_planner_environment.h>

#include <l3_footstep_planning_plugins/aggregator/state_generator.h>

namespace l3_footstep_planning
{
FootstepPlannerEnvironment::FootstepPlannerEnvironment(const EnvironmentParameters& params, PlanningFeedbackHandler& feedback_handler)
  : DiscreteSpaceInformation()
  , params_(params)
  , state_space_(new StateSpace(params))
  , feedback_handler_(feedback_handler)
  , num_expanded_states_(0)
  , num_visited_states_(0)
{
}

void FootstepPlannerEnvironment::reset()
{
  state_space_->reset();
  num_expanded_states_ = 0;
  num_visited_states_ = 0;
}

int FootstepPlannerEnvironment::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  PlanningState::ConstPtr from = StateSpaceManager::getPlanningState(static_cast<UID>(FromStateID));
  PlanningState::ConstPtr to = StateSpaceManager::getPlanningState(static_cast<UID>(ToStateID));

  ROS_ASSERT(from);
  ROS_ASSERT(to);

  return GetFromToHeuristic(from, to, state_space_->getStartPlanningState(), state_space_->getGoalPlanningState());
}

int FootstepPlannerEnvironment::GetFromToHeuristic(PlanningState::ConstPtr from, PlanningState::ConstPtr to, PlanningState::ConstPtr start, PlanningState::ConstPtr goal)
{
  return cvMmScale * params_.heuristic_scale * Heuristic::instance().getHeuristicValue(*from->getState(), *to->getState(), *start->getState(), *goal->getState());
}

int FootstepPlannerEnvironment::GetStartHeuristic(int stateID)
{
  PlanningState::ConstPtr current = StateSpaceManager::getPlanningState(static_cast<UID>(stateID));
  ROS_ASSERT(current);
  return GetFromToHeuristic(state_space_->getStartPlanningState(), current, state_space_->getStartPlanningState(), state_space_->getGoalPlanningState());
}

int FootstepPlannerEnvironment::GetGoalHeuristic(int stateID)
{
  PlanningState::ConstPtr current = StateSpaceManager::getPlanningState(static_cast<UID>(stateID));
  ROS_ASSERT(current);
  return GetFromToHeuristic(current, state_space_->getGoalPlanningState(), state_space_->getStartPlanningState(), state_space_->getGoalPlanningState());
}

void FootstepPlannerEnvironment::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV)
{
  boost::this_thread::interruption_point();

  PredIDV->clear();
  CostV->clear();

  UID target_state_uid = static_cast<UID>(TargetStateID);

  ROS_ASSERT(target_state_uid > 0 && target_state_uid <= StateSpaceManager::getNumPlanningStates());

  PlanningStateHashed::ConstPtr current = StateSpaceManager::getPlanningState(target_state_uid);

  // make start states always absorbing
  if (current->getState()->isStart())
    return;

  ROS_ASSERT(current);

  if (!current->getSuccState())
  {
    ROS_WARN("State should be expanded but current state has no successor!");
    return;
  }

  // make sure start state transitions are consistent with
  // GetPreds(some_state, start_state) where some_state is reachable by an
  // arbitrary step from start_state
  if (params_.forward_search)
  {
    ROS_WARN("GetPreds called during forward search.");
  }

  stateExpanded(current);

  // sample states
  std::list<PlanningStateGenResult> gen_states = StateGenerator::instance().generatePredecessors(current, state_space_->getStartState(), state_space_->getGoalState());

  // sample states with start footholds (Note: This helps to converge faster as regular sampling may miss the start state due to discretization errors)
  gen_states.splice(gen_states.end(), StateGenerator::instance().generateNearStartStates(current, state_space_->getStartPlanningState(), state_space_->getGoalPlanningState()));

  PredIDV->reserve(gen_states.size());
  CostV->reserve(gen_states.size());

  for (const PlanningStateGenResult& result : gen_states)
  {
    // check for start state
    if (result.state->getState()->isStart())
      PredIDV->push_back(state_space_->getStartPlanningState()->getUID()); // push uid of start state
    else
      PredIDV->push_back(result.state->getUID());

    CostV->push_back(static_cast<int>(cvMmScale * result.cost + 0.5));
    stateVisited(result.state);
  }
}

void FootstepPlannerEnvironment::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV)
{
  boost::this_thread::interruption_point();

  SuccIDV->clear();
  CostV->clear();

  UID source_state_uid = static_cast<UID>(SourceStateID);

  ROS_ASSERT(source_state_uid > 0 && source_state_uid <= StateSpaceManager::getNumPlanningStates());

  PlanningStateHashed::ConstPtr current = StateSpaceManager::getPlanningState(source_state_uid);

  // make goal states always absorbing
  if (current->getState()->isGoal())
    return;

  ROS_ASSERT(current);

  if (!current->getPredState())
  {
    ROS_WARN("State should be expanded but current state has no predecessor!");
    return;
  }

  // make sure start state transitions are consistent with
  // GetPreds(some_state, start_state) where some_state is reachable by an
  // arbitrary step from start_state
  if (!params_.forward_search)
  {
    ROS_WARN("GetSuccs called during backward search.");
  }

  stateExpanded(current);

  // sample states
  std::list<PlanningStateGenResult> gen_states = StateGenerator::instance().generateSuccessors(current, state_space_->getStartState(), state_space_->getGoalState());

  // sample states with goal footholds (Note: This helps to converge faster as regular sampling may miss the goal state due to discretization errors)
  gen_states.splice(gen_states.end(), StateGenerator::instance().generateNearGoalStates(current, state_space_->getStartPlanningState(), state_space_->getGoalPlanningState()));

  SuccIDV->reserve(gen_states.size());
  CostV->reserve(gen_states.size());
  for (const PlanningStateGenResult& result : gen_states)
  {
    // check for goal state
    if (result.state->getState()->isGoal())
      SuccIDV->push_back(state_space_->getGoalPlanningState()->getUID()); // push uid of goal state
    else
      SuccIDV->push_back(result.state->getUID());

    CostV->push_back(static_cast<int>(cvMmScale * result.cost + 0.5));
    stateVisited(result.state);
  }
}

void FootstepPlannerEnvironment::GetRandomPredsatDistance(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CLowV)
{
  boost::this_thread::interruption_point();

  PredIDV->clear();
  CLowV->clear();

  UID target_state_uid = static_cast<UID>(TargetStateID);

  ROS_ASSERT(target_state_uid > 0 && target_state_uid <= StateSpaceManager::getNumPlanningStates());

  PlanningStateHashed::ConstPtr current = StateSpaceManager::getPlanningState(target_state_uid);

  // make start states always absorbing
  if (current->getState()->isStart())
    return;

  ROS_ASSERT(current);

  if (!current->getSuccState())
  {
    ROS_WARN("State should be expanded but current state has no successor!");
    return;
  }

  // make sure start state transitions are consistent with
  // GetPreds(some_state, start_state) where some_state is reachable by an
  // arbitrary step from start_state
  if (params_.forward_search)
  {
    ROS_WARN("GetRandomPredsatDistance called during forward search.");
  }

  stateExpanded(current);

  // sample states
  std::list<PlanningStateGenResult> gen_states = StateGenerator::instance().generateRandomPredecessors(current, params_.num_random_nodes, state_space_->getStartState(), state_space_->getGoalState());

  // sample states with start footholds (Note: This helps to converge faster as regular sampling may miss the goal state due to discretization errors)
  gen_states.splice(gen_states.end(), StateGenerator::instance().generateNearStartStates(current, state_space_->getStartPlanningState(), state_space_->getGoalPlanningState()));

  PredIDV->reserve(gen_states.size());
  CLowV->reserve(gen_states.size());
  for (const PlanningStateGenResult& result : gen_states)
  {
    // check for start state
    if (result.state->getState()->isStart())
      PredIDV->push_back(state_space_->getStartPlanningState()->getUID()); // push uid of start state
    else
      PredIDV->push_back(result.state->getUID());

    CLowV->push_back(GetFromToHeuristic(state_space_->getStartPlanningState(), result.state, state_space_->getStartPlanningState(), state_space_->getGoalPlanningState()));
    stateVisited(result.state);
  }
}

void FootstepPlannerEnvironment::GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV)
{
  boost::this_thread::interruption_point();

  SuccIDV->clear();
  CLowV->clear();

  UID source_state_uid = static_cast<UID>(SourceStateID);

  ROS_ASSERT(source_state_uid > 0 && source_state_uid <= StateSpaceManager::getNumPlanningStates());

  PlanningStateHashed::ConstPtr current = StateSpaceManager::getPlanningState(source_state_uid);

  // make goal states always absorbing
  if (current->getState()->isGoal())
    return;

  ROS_ASSERT(current);

  if (!current->getPredState())
  {
    ROS_WARN("State should be expanded but current state has no predecessor!");
    return;
  }

  // make sure start state transitions are consistent with
  // GetPreds(some_state, start_state) where some_state is reachable by an
  // arbitrary step from start_state
  if (!params_.forward_search)
  {
    ROS_WARN("GetRandomSuccsatDistance called during backward search.");
  }

  stateExpanded(current);

  // sample states
  std::list<PlanningStateGenResult> gen_states = StateGenerator::instance().generateRandomSuccessors(current, params_.num_random_nodes, state_space_->getStartState(), state_space_->getGoalState());

  // sample states with goal footholds (Note: This helps to converge faster as regular sampling may miss the goal state due to discretization errors)
  gen_states.splice(gen_states.end(), StateGenerator::instance().generateNearGoalStates(current, state_space_->getStartPlanningState(), state_space_->getGoalPlanningState()));

  SuccIDV->reserve(gen_states.size());
  CLowV->reserve(gen_states.size());
  for (const PlanningStateGenResult& result : gen_states)
  {
    // check for goal state
    if (result.state->getState()->isGoal())
      SuccIDV->push_back(state_space_->getGoalPlanningState()->getUID()); // push uid of goal state
    else
      SuccIDV->push_back(result.state->getUID());

    CLowV->push_back(GetFromToHeuristic(result.state, state_space_->getGoalPlanningState(), state_space_->getStartPlanningState(), state_space_->getGoalPlanningState()));
    stateVisited(result.state);
  }
}

bool FootstepPlannerEnvironment::AreEquivalent(int StateID1, int StateID2)
{
  if (StateID1 == StateID2)
    return true;

  PlanningState::ConstPtr s1 = StateSpaceManager::getPlanningState(static_cast<UID>(StateID1));
  PlanningState::ConstPtr s2 = StateSpaceManager::getPlanningState(static_cast<UID>(StateID2));

  ROS_ASSERT(s1);
  ROS_ASSERT(s2);

  return s1->getState()->getUID() == s2->getState()->getUID();
}

bool FootstepPlannerEnvironment::InitializeEnv(const char* sEnvFile)
{
  //  ROS_ERROR("FootstepPlanerEnvironment::InitializeEnv: Hit unimplemented "
  //            "function. Check this!");
  return true;
}

bool FootstepPlannerEnvironment::InitializeMDPCfg(MDPConfig* MDPCfg)
{
  MDPCfg->startstateid = static_cast<int>(state_space_->getStartPlanningState()->getUID());
  MDPCfg->goalstateid = static_cast<int>(state_space_->getGoalPlanningState()->getUID());

  return true;
}

void FootstepPlannerEnvironment::PrintEnv_Config(FILE* fOut)
{
  // NOTE: implement this if the planner needs to print out configurations
  ROS_ERROR("FootstepPlanerEnvironment::PrintEnv_Config: Hit "
            "unimplemented function. Check this!");
}

void FootstepPlannerEnvironment::PrintState(int stateID, bool bVerbose, FILE* fOut)
{
  if (fOut == nullptr)
    fOut = stdout;

  if (stateID == state_space_->getGoalPlanningState()->getUID() && bVerbose)
    SBPL_FPRINTF(fOut, "the state is a goal state\n");

  PlanningState::ConstPtr s = StateSpaceManager::getPlanningState(static_cast<UID>(stateID));

  for (Foothold::ConstPtr f : s->getState()->getFootholds())
    SBPL_FPRINTF(fOut, "%.2f %.2f %.2f %.2f %.2f %.2f |", f->x(), f->y(), f->z(), f->roll(), f->pitch(), f->yaw());
  SBPL_FPRINTF("\n");
}

void FootstepPlannerEnvironment::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  // NOTE: not implemented so far
  // Description: Some searches may also use SetAllActionsandAllOutcomes
  // or SetAllPreds functions if they keep the pointers to successors
  // (predecessors) but most searches do not require this, so it is not
  // necessary to support this

  ROS_ERROR("FootstepPlannerEnvironment::SetAllActionsandAllOutcomes: Hit"
            " unimplemented function. Check this!");
}

void FootstepPlannerEnvironment::SetAllPreds(CMDPSTATE* state)
{
  // NOTE: not implemented so far
  // Description: Some searches may also use SetAllActionsandAllOutcomes
  // or SetAllPreds functions if they keep the pointers to successors
  // (predecessors) but most searches do not require this, so it is not
  // necessary to support this

  ROS_ERROR("FootstepPlannerEnvironment::SetAllPreds: Hit unimplemented "
            "function. Check this!");
}
}  // namespace l3_footstep_planning
