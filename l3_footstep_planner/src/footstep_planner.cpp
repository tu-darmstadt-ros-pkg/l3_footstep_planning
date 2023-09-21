#include <l3_footstep_planner/footstep_planner.h>

#include <l3_plugins/robot_model.h>

#include <l3_libs/conversions/l3_msg_conversions.h>

#include <l3_footstep_planning_libs/modeling/state_space_manager.h>

#include <l3_footstep_planning_plugins/aggregator/use_mask_generator.h>
#include <l3_footstep_planning_plugins/aggregator/step_cost_estimator.h>
#include <l3_footstep_planning_plugins/aggregator/heuristic.h>
#include <l3_footstep_planning_plugins/aggregator/post_processor.h>
#include <l3_footstep_planning_plugins/aggregator/reachability.h>
#include <l3_footstep_planning_plugins/aggregator/state_generator.h>
#include <l3_footstep_planning_plugins/aggregator/world_model.h>
#include <l3_footstep_planning_plugins/aggregator/hfs_heuristic.h>

#include <l3_footstep_planner/pattern_planner.h>

#include <l3_footstep_planning_tools/foot_pose_transformer.h>

namespace l3_footstep_planning
{
FootstepPlanner::FootstepPlanner(ros::NodeHandle& nh)
  : feedback_handler_(nh)
  , start_foot_idx_(msgs::StepPlanRequest::AUTO_START_FOOT_IDX)
  , start_pose_set_up_(false)
  , goal_pose_set_up_(false)
  , path_cost_(0)
  , step_plan_uid_(0)
{
  // initialize the planner environment
  if (!ParameterManager::empty())
    setParams(ParameterManager::getActive());
  else
    ROS_ERROR("[FootstepPlanner] Can't initialize environment due to missing parameters!");

  // initialize foot pose transformer
  if (!RobotModel::description())
  {
    ROS_ERROR("[FootstepPlanner] No robot description available!");
    return;
  }

  FootPoseTransformer::initialize(RobotModel::description());
}

FootstepPlanner::~FootstepPlanner()
{
  boost::recursive_mutex::scoped_lock lock(planner_mutex_);

  planning_thread_.interrupt();
  planning_thread_.join();
}

void FootstepPlanner::setPlanner()
{
  /// @TODO: fix called twice during reset
  if (env_params_->planner_type == "ARAPlanner" || env_params_->planner_type == "ADPlanner" || env_params_->planner_type == "ANAPlanner" ||
      env_params_->planner_type == "RSTARPlanner")
    ROS_INFO_STREAM("Planning with " << env_params_->planner_type);
  else
    ROS_ERROR_STREAM("Planner " << env_params_->planner_type << " not available / untested.");

  if (env_params_->forward_search)
    ROS_INFO_STREAM("Search direction: forward planning");
  else
    ROS_INFO_STREAM("Search direction: backward planning");

  if (env_params_->planner_type == "ARAPlanner")
    planner_.reset(new ARAPlanner(planner_environment_.get(), env_params_->forward_search));
  else if (env_params_->planner_type == "ADPlanner")
    planner_.reset(new ADPlanner(planner_environment_.get(), env_params_->forward_search));
  else if (env_params_->planner_type == "RSTARPlanner")
  {
    planner_.reset(new RSTARPlanner(planner_environment_.get(), env_params_->forward_search));
    // new options, require patched SBPL
    //          planner_->set_local_expand_thres(500);
    //          planner_->set_eps_step(1.0);
  }
  else if (env_params_->planner_type == "ANAPlanner")
    planner_.reset(new anaPlanner(planner_environment_.get(), env_params_->forward_search));
}

bool FootstepPlanner::isPlanning() const { return planning_thread_.joinable(); }

bool FootstepPlanner::plan(ReplanParams& params)
{
  int ret = 0;
  MDPConfig mdp_config;
  std::vector<int> solution_state_ids;

  // commit start/goal poses to the environment
  UID start_uid = planner_environment_->getStateSpace()->updateStart(*start_state_);
  UID goal_uid = planner_environment_->getStateSpace()->updateGoal(*goal_state_);
  planner_environment_->getStateSpace()->setStartFootIndex(start_foot_idx_);
  planner_environment_->InitializeEnv(NULL);
  planner_environment_->InitializeMDPCfg(&mdp_config);

  // inform AD planner about changed (start) states for replanning
  if (pathExists() && !env_params_->forward_search && env_params_->planner_type == "ADPlanner")
  {
    std::vector<int> changed_edges;
    changed_edges.push_back(mdp_config.startstateid);
    // update the AD planner
    l3::SharedPtr<ADPlanner> ad_planner = l3::dynamicPointerCast<ADPlanner>(planner_);
    ad_planner->update_preds_of_changededges(&changed_edges);
  }

  // set up SBPL
  if (planner_->set_start(mdp_config.startstateid) == 0)
  {
    ROS_ERROR("Failed to set start state.");
    return false;
  }
  if (planner_->set_goal(mdp_config.goalstateid) == 0)
  {
    ROS_ERROR("Failed to set goal state");
    return false;
  }

  planner_->set_initialsolution_eps(params.initial_eps);
  planner_->set_search_mode(params.return_first_solution);

  int path_cost;
  try
  {
    ROS_INFO("Start planning (max time: %f, eps_0: %.2f, d_eps: %.2f, h(start, goal): %.3f)", params.max_time, params.initial_eps, params.dec_eps,
             (double)planner_environment_->GetGoalHeuristic(planner_environment_->getStateSpace()->getStartPlanningState()->getUID()) / (double)cvMmScale);
    ret = planner_->replan(params.max_time, &solution_state_ids, &path_cost);
  }
  catch (const SBPL_Exception* e)
  {
    ROS_ERROR("SBPL planning failed (%s)", e->what());
    return false;
  }
  path_cost_ = double(path_cost) / cvMmScale;

  ROS_INFO("Expanded states: %i total / %i new", planner_environment_->getNumExpandedStates(), planner_->get_n_expands());
  ROS_INFO("Generated footholds: %lu", StateSpaceManager::getNumFootholds());
  ROS_INFO("Generated floating bases: %lu", StateSpaceManager::getNumFloatingBases());
  ROS_INFO("Generated states: %lu", StateSpaceManager::getNumStates());
  ROS_INFO("Generated planning states: %lu", StateSpaceManager::getNumPlanningStates());
  // ROS_INFO("Generated transitions: %lu", StateSpaceManager::getNumTransitions());

  if (ret && solution_state_ids.size() > 0)
  {
    ROS_INFO("Initial solution found after %fs", planner_->get_initial_eps_planning_time());
    ROS_INFO("Solution of size %zu found after %fs", solution_state_ids.size(), planner_->get_final_eps_planning_time());

    unsigned long hits = StateSpaceManager::footholdHits();
    unsigned long miss = StateSpaceManager::footholdMiss();
    ROS_INFO("Foothold DB hit rate %.2f (Hits: %lu / Misses: %lu)", static_cast<double>(hits) / static_cast<double>(hits + miss), hits, miss);

    hits = StateSpaceManager::stateHits();
    miss = StateSpaceManager::stateMiss();
    ROS_INFO("State DB hit rate %.2f (Hits: %lu / Misses: %lu)", static_cast<double>(hits) / static_cast<double>(hits + miss), hits, miss);

    hits = StateSpaceManager::planningStatesHits();
    miss = StateSpaceManager::planningStatesMiss();
    ROS_INFO("Planning State DB hit rate %.2f (Hits: %lu / Misses: %lu)", static_cast<double>(hits) / static_cast<double>(hits + miss), hits, miss);

    if (extractPath(solution_state_ids, start_uid, goal_uid))
    {
      ROS_INFO("Final eps: %f", planner_->get_final_epsilon());
      ROS_INFO("Path cost: %f (%i)\n", path_cost_, path_cost);
      return true;
    }
    else
    {
      ROS_ERROR("extracting path failed\n\n");
      return false;
    }
  }
  else
  {
    ROS_ERROR("No solution found");
    return false;
  }
}

bool FootstepPlanner::extractPath(const std::vector<int>& state_ids, const UID& start_uid, const UID& goal_uid)
{
  path_.clear();

  if (state_ids.empty())
    return false;

  StepIndex step_idx = 0;
  PlanningState::ConstPtr prev_pstate;

  for (const int uid : state_ids)
  {
    PlanningState::ConstPtr pstate = StateSpaceManager::getPlanningState(static_cast<UID>(uid));
    if (!pstate)
    {
      path_.clear();
      return false;
    }

    Step::Ptr step;

    if (static_cast<UID>(uid) == start_uid)
    {
      // start state consists of all "non-moving step data" of all feet and all floating bases
      step = Step::make();
      for (Foothold::ConstPtr fh : pstate->getState()->getFootholds())
        step->footStep().updateNonMovingLink(fh->idx, fh);
      for (FloatingBase::ConstPtr fb : pstate->getState()->getFloatingBases())
        step->baseStep().updateNonMovingLink(fb->idx, fb);
    }
    else if (static_cast<UID>(uid) == goal_uid)
    {
      // goal state should not be the first state
      if (path_.empty())
        return false;

      // reconstruct step
      if (env_params_->forward_search)
      {
        PlanningState temp = PlanningState(pstate->getState(), prev_pstate->getState());
        PostProcessor::instance().postProcess(temp);
        pstate = StateSpaceManager::createPlanningState(temp);
      }
      else
      {
        /// @todo How does the goal state look when backward planning?
        PlanningState temp = PlanningState(prev_pstate->getState(), StateHashed::ConstPtr(), pstate->getState());
        PostProcessor::instance().postProcess(temp);
        // pstate = StateSpaceManager::getPlanningState(temp);
      }

      ROS_ASSERT(pstate);
      step = Step::make(*pstate->getStep());
    }
    else
    {
      step = Step::make(*pstate->getStep());
    }

    step->setStepIndex(step_idx++);
    path_.push_back(step);

    prev_pstate = pstate;
  }

  return true;
}

bool FootstepPlanner::setParams(const vigir_generic_params::ParameterSet& params)
{
  boost::recursive_mutex::scoped_lock lock(planner_mutex_);

  // reinitialize the planner environment parameters
  env_params_.reset(new EnvironmentParameters(params));

  // update feedback handler
  feedback_handler_.setParameters(*env_params_);

  // get params
  print_use_masks_ = params.param("print_use_masks", false, true);
  print_solution_ = params.param("print_solution", false, true);

  // load plugin set
  std::string plugin_set_name;
  if (params.getParam("plugin_set", plugin_set_name))
    vigir_pluginlib::PluginManager::loadPluginSet(plugin_set_name);
  else
    ROS_WARN("[FootstepPlanner] setParams: No plugin set was given by parameter set '%s'", params.getName().c_str());

  // reinitialize StateManager
  StateSpaceManager::initialize(params);

  /// @todo: Notify vis node of new plugin set

  // hold already loaded plugins, as the configured plugins could be overwritten/removed by the plugin set
  // except no plugins are loaded
  if (!RobotModel::gaitGenerator())
  {
    ROS_WARN("[FootstepPlanner] RobotModelServer does not provide any get generator. Using locally defined GaitGenerator.");
    RobotModel::loadGaitGeneratorPlugin();
    RobotModel::printPluginSummary();
  }

  // reinitialize use mask generator
  UseMaskGenerator::mutableInstance().loadPlugins(false);
  UseMaskGenerator::mutableInstance().loadParams(params);

  // reinitialize state generator
  StateGenerator::mutableInstance().loadPlugins();
  StateGenerator::mutableInstance().loadParams(params);

  // reinitialize reachability models
  Reachability::mutableInstance().loadPlugins();
  Reachability::mutableInstance().loadParams(params);

  // reinitialize post processor
  PostProcessor::mutableInstance().loadPlugins(false);
  PostProcessor::mutableInstance().loadParams(params);

  // reinitialize world model
  WorldModel::mutableInstance().loadPlugins(false);
  WorldModel::mutableInstance().loadParams(params);

  // reinitialize step cost estimators
  StepCostEstimator::mutableInstance().loadPlugins();
  StepCostEstimator::mutableInstance().loadParams(params);

  // reinitialize heuristics
  Heuristic::mutableInstance().loadPlugins();
  Heuristic::mutableInstance().loadParams(params);

  // reinitialize hfs heuristics
  HFSHeuristic::mutableInstance().loadPlugins(false);
  HFSHeuristic::mutableInstance().loadParams(params);

  // print use mask summary
  // clang-format off
  if (print_use_masks_)
  {
    ROS_INFO("Plugin UseMask:");
    std::vector<FootstepPlanningPlugin::ConstPtr> plugins;
    vigir_pluginlib::PluginManager::getPlugins(plugins);
    for (FootstepPlanningPlugin::ConstPtr p : plugins)
    {
      const UseMask& use_mask = p->getUseMask();

      std::string color;
      if (use_mask & USE_ALWAYS)
        color = "\033[32m"; // green
      else if (!use_mask)
        color = "\033[31m"; // red
      else
        color = "\033[33m"; // yellow

      ROS_INFO_STREAM(color << "    [" << std::bitset<32>(use_mask) << "] (" << use_mask << ") " << p->getName());
    }
  }

  // clang-format on

  resetTotally();

  return true;
}

msgs::ErrorStatus FootstepPlanner::updateFoot(msgs::Foothold& foot, uint8_t mode, bool transform) const
{
  msgs::ErrorStatus status;

  // transform to sole frame
  if (transform)
    FootPoseTransformer::transformToPlannerFrame(foot);

  if (mode & msgs::UpdateMode::UPDATE_MODE_MOVE_TO_VALID)
  {
    Foothold f(foot);
    if (findNearestValidFoothold(f))
      f.toMsg(foot);
    else
      status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner", "updateFoot: Couldn't determine valid position!");
  }
  else if (mode & msgs::UpdateMode::UPDATE_MODE_3D)
  {
    TerrainResult result;

    Foothold f(foot);
    if (WorldModel::instance().isTerrainModelAvailable())
    {
      result = WorldModel::instance().getTerrainModel()->update3DData(f);
      if ((result & TerrainResult::NO_DATA) == TerrainResult::NO_DATA)
        status += ErrorStatusWarning(msgs::ErrorStatus::WARN_NO_TERRAIN_DATA, "FootstepPlanner", "updateFoot: Couldn't update 3D data.", false);
      else
        f.toMsg(foot);
    }
    else
      status += ErrorStatusWarning(msgs::ErrorStatus::WARN_NO_TERRAIN_DATA, "FootstepPlanner", "updateFoot: Couldn't update 3D data. No terrain data available.", false);
  }
  else if (mode & msgs::UpdateMode::UPDATE_MODE_Z)
  {
    if (WorldModel::instance().isTerrainModelAvailable())
    {
      if (WorldModel::instance().getTerrainModel()->getHeight(foot.pose.position.x, foot.pose.position.y, foot.pose.position.z) != TerrainResult::OK)
        status += ErrorStatusWarning(msgs::ErrorStatus::WARN_NO_TERRAIN_DATA, "FootstepPlanner", "updateFoot: Couldn't update foot height.", false);
    }
    else
      status += ErrorStatusWarning(msgs::ErrorStatus::WARN_NO_TERRAIN_DATA, "FootstepPlanner", "updateFoot: Couldn't update foot height. No terrain data available.", false);
  }

  // transform back to robot frame
  if (transform)
    FootPoseTransformer::transformToRobotFrame(foot);

  foot.header.stamp = ros::Time::now();

  return status;
}

msgs::ErrorStatus FootstepPlanner::updateFeet(msgs::FootholdArray& footholds, uint8_t mode, bool transform) const
{
  msgs::ErrorStatus status;

  if (mode & msgs::UpdateMode::UPDATE_MODE_MOVE_TO_VALID)
  {
    FootholdHashedConstPtrArray footholds_hashed;

    FootholdArray footholds_l3;
    footholdArrayMsgToL3(footholds, footholds_l3);

    UID fake_uid = 0;
    bool success = true;
    for (Foothold& f : footholds_l3)
    {
      if (!findNearestValidFoothold(f))
      {
        status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner",
                                   "updateFeet: Couldn't determine valid feet position for foot '" + RobotModel::description()->getFootInfo(f.idx).name + "' (" +
                                       boost::lexical_cast<std::string>(f.idx) + ")!");
        success = false;
        break;
      }
      else
        footholds_hashed.push_back(FootholdHashed::make(f, fake_uid++));
    }

    if (success)
    {
      if (Reachability::instance().isReachable(State(footholds_hashed)))
        footholdArrayL3ToMsg(footholds_l3, footholds);
      else
        status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner", "updateFeet: Couldn't determine valid position!");
    }

    mode -= msgs::UpdateMode::UPDATE_MODE_MOVE_TO_VALID;
  }

  for (msgs::Foothold& f : footholds)
    status += updateFoot(f, mode, transform);

  return status;
}

msgs::ErrorStatus FootstepPlanner::updateStepPlan(msgs::StepPlan& step_plan, uint8_t mode, const std::string& param_set_name, bool transform) const
{
  if (step_plan.plan.steps.empty())
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner", "updateStepPlan: Can't process empty step plan!");
  if (step_plan.start.empty())
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner", "updateStepPlan: Can't process step plan due to missing start feet poses!");

  // lock entire planning system
  boost::recursive_mutex::scoped_lock lock(planner_mutex_);

  // transform to sole frame
  if (transform)
    FootPoseTransformer::transformToPlannerFrame(step_plan);

  msgs::ErrorStatus status;

  if (mode & msgs::UpdateMode::UPDATE_MODE_REPLAN)
  {
    ROS_WARN("UPDATE_MODE_REPLAN isn't implemented yet!");
  }
  else
  {
    /// @TODO: use RobotModelPlugin
    FootholdArray footholds;
    footholdArrayMsgToL3(step_plan.start, footholds);
    State::ConstPtr start_state = StateSpaceManager::createState(footholds, FloatingBaseArray());

    std::vector<msgs::Step>::iterator itr = step_plan.plan.steps.begin();
    // State prev_state = State(StepData(*itr));

    itr++;
    for (; itr != step_plan.plan.steps.end(); itr++)
    {
      msgs::Step& cur_step = *itr;
      // State cur_state = State(StepData(cur_step));

      for (msgs::FootStepData& cur_foot_step : cur_step.foot_steps)
      {
        // update feet
        status += updateFoot(cur_foot_step.origin, mode, false);
        status += updateFoot(cur_foot_step.target, mode, false);

        // check reachability
        if (mode & msgs::UpdateMode::UPDATE_MODE_CHECK_VALIDITY)
        {
          /// @TODO: Reimplemented support for UPDATE_MODE_CHECK_VALIDITY
          // cur_step_data.valid = RobotModel::instance().isReachable(left_state, right_state, cur_state);
        }

        // check collision
        if (mode & msgs::UpdateMode::UPDATE_MODE_CHECK_COLLISION)
        {
          /// @TODO: Reimplemented support for UPDATE_MODE_CHECK_COLLISION
          // cur_step_data.colliding = !WorldModel::instance().isAccessible(cur_state, prev_state);
        }

        // recompute cost
        if (mode & msgs::UpdateMode::UPDATE_MODE_COST)
        {
          /// @TODO: Reimplemented support for UPDATE_MODE_COST
          //        double c, r;
          //        if (StepCostEstimator::instance().getCost(left_state, right_state, cur_state, c, r))
          //        {
          //          /// @TODO
          //          cur_step.cost = c;
          //          cur_step.risk = r;
          //        }
          //        else
          //          status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlanner", "updateStepPlan: Couldn't determine cost for step " +
          //          boost::lexical_cast<std::string>(cur_step.step_idx) + "!");

          status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner", "updateStepPlan: Cost update not implemented yet!");
        }

        // prepare next iteration
        /// @TODO: use RobotModelPlugin
        //      if (cur_state.getFootIndex() == 1)
        //        left_state = cur_state;
        //      else
        //        right_state = cur_state;

        //      prev_state = cur_state;
      }
    }
  }

  // transform back to robot frame
  if (transform)
    FootPoseTransformer::transformToRobotFrame(step_plan);

  return msgs::ErrorStatus();
}

void FootstepPlanner::reset()
{
  boost::recursive_mutex::scoped_lock lock(planner_mutex_);

  Heuristic::mutableInstance().resetPlugins();

  // reset the previously calculated paths
  path_.clear();
  // reset the planner
  // INFO: force_planning_from_scratch was not working properly the last time
  // checked; therefore instead of using this function the planner is manually
  // reset
  // ivPlannerPtr->force_planning_from_scratch();
  feedback_handler_.reset();
  planner_environment_->reset();
  StateSpaceManager::clear();
  setPlanner();
}

void FootstepPlanner::resetTotally()
{
  boost::recursive_mutex::scoped_lock lock(planner_mutex_);

  // reset plugins
  UseMaskGenerator::mutableInstance().resetPlugins();
  StateGenerator::mutableInstance().resetPlugins();
  Reachability::mutableInstance().resetPlugins();
  PostProcessor::mutableInstance().resetPlugins();
  StepCostEstimator::mutableInstance().resetPlugins();
  Heuristic::mutableInstance().resetPlugins();
  HFSHeuristic::mutableInstance().resetPlugins();

  // reset state manager
  StateSpaceManager::clear();

  // reset the previously calculated paths
  path_.clear();
  // reinitialize the planner environment
  feedback_handler_.reset();
  planner_environment_.reset(new FootstepPlannerEnvironment(*env_params_, feedback_handler_));
  setPlanner();

  StateSpaceManager::setSbplEnvironment(planner_environment_->StateID2IndexMapping);
}

msgs::ErrorStatus FootstepPlanner::planSteps(msgs::StepPlanRequestService::Request& req)
{
  ReplanParams params(req.plan_request.max_planning_time > 0 ? req.plan_request.max_planning_time : env_params_->max_planning_time);
  params.initial_eps = req.plan_request.initial_eps > 0.0 ? req.plan_request.initial_eps : env_params_->initial_eps;
  params.final_eps = 1.0;
  params.dec_eps = env_params_->decrease_eps;
  params.return_first_solution = env_params_->search_until_first_solution;
  params.repair_time = -1;

  // start the planning and return success
  if (!plan(params))
    return ErrorStatusError(msgs::ErrorStatus::ERR_NO_SOLUTION, "FootstepPlanner", "planSteps: No solution found!");

  return msgs::ErrorStatus();
}

msgs::ErrorStatus FootstepPlanner::planPattern(msgs::StepPlanRequestService::Request& req)
{
  // set start foot poses
  if (!setStart(req.plan_request, true))
    return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_START, "FootstepPlanner", "planPattern: Could not set start pose! Please check if poses are set!");

  PatternPlanner p;
  return p.planPattern(path_, start_state_, req);
}

bool FootstepPlanner::finalizeStepPlan(msgs::StepPlanRequestService::Request& req, msgs::StepPlanRequestService::Response& resp, bool post_process)
{
  /// @TODO: Use native l3:StepPlan structure and convert it to msg
  resp.step_plan.header.frame_id = req.plan_request.header.frame_id;
  resp.step_plan.header.stamp = req.plan_request.header.stamp;
  resp.step_plan.header.seq = step_plan_uid_++;

  // add footstep plan
  msgs::FootStepData foot_step_msg;

  footholdArrayL3ToMsg(start_state_->getFootholds(), resp.step_plan.start);
  if (goal_state_)
    footholdArrayL3ToMsg(goal_state_->getFootholds(), resp.step_plan.goal);

  foot_step_msg.origin.header = resp.step_plan.header;
  foot_step_msg.target.header = resp.step_plan.header;
  //  step.valid = true;
  //  step.colliding = false;

  StepIndex start_step_idx = req.plan_request.start_step_idx;
  resp.step_plan.plan.header = resp.step_plan.header;
  resp.step_plan.plan.steps.reserve(getPathSize());

  for (Step::Ptr step : path_)
  {
    /// @TODO: use RobotModelPlugin

    // post process step (needed for goal states and pattern mode)
    //      if ((goal_state_ && !goal_state_->getFootholds().empty() && (swing_foot == State(*goal_state_->getFoothold(0)) || swing_foot == State(*goal_state_->getFoothold(1))))
    //      ||
    //          (path_iter != getPathBegin() && req.plan_request.planning_mode == msgs::StepPlanRequest::PLANNING_MODE_PATTERN))
    //      {
    //        if (env_params->forward_search)
    //          PostProcessor::instance().postProcessForward(left_foot, right_foot, swing_foot);
    //        else
    //          PostProcessor::instance().postProcessBackward(left_foot, right_foot, swing_foot);
    //      }

    // get start and goal foothold config
    for (const Step::FootStep::MovingDataPair& p : step->footStep().getMovingLinks())
    {
      FootStepData::Ptr foot_step = p.second;

      // convert footstep
      Foothold fh = *foot_step->origin;
      fh.header = resp.step_plan.header;
      foot_step->origin = makeShared<Foothold>(fh);

      fh = *foot_step->target;
      fh.header = resp.step_plan.header;
      foot_step->target = makeShared<Foothold>(fh);
    }

    // get non-moving legs
    for (const Step::FootStep::NonMovingDataPair& p : step->footStep().getNonMovingLinks())
    {
      // convert footstep
      Foothold::Ptr fh = makeShared<Foothold>(*p.second);
      fh->header = resp.step_plan.header;
      step->footStep().updateNonMovingLink(fh->idx, fh);
    }

    // get start and goal floating base config
    for (const Step::BaseStep::MovingDataPair& p : step->baseStep().getMovingLinks())
    {
      BaseStepData::Ptr base_step_data = p.second;

      // convert floating base
      FloatingBase fb = *base_step_data->origin;
      fb.header = resp.step_plan.header;
      base_step_data->origin = makeShared<FloatingBase>(fb);

      fb = *base_step_data->target;
      fb.header = resp.step_plan.header;
      base_step_data->target = makeShared<FloatingBase>(fb);
    }

    // get non-moving floating bases
    for (const Step::BaseStep::NonMovingDataPair& p : step->baseStep().getNonMovingLinks())
    {
      // convert floating base
      FloatingBase::Ptr fb(new FloatingBase(*p.second));
      fb->header = resp.step_plan.header;
      step->baseStep().updateNonMovingLink(fb->idx, fb);
    }

    step->setStepIndex(step->getStepIndex() + start_step_idx);

    // finally add step to plan
    resp.step_plan.plan.steps.push_back(step->toMsg());
  }

  // perform post processing on entire plan
  StepPlan plan(resp.step_plan);
  PostProcessor::instance().postProcess(plan);
  resp.step_plan = plan.toMsg();

  // plan validation and computation of final cost
  if (req.plan_request.planning_mode == msgs::StepPlanRequest::PLANNING_MODE_PATTERN)
    updateStepPlan(resp.step_plan, msgs::UpdateMode::UPDATE_MODE_CHECK_VALIDITY | msgs::UpdateMode::UPDATE_MODE_CHECK_COLLISION /*| msgs::UpdateMode::UPDATE_MODE_COST*/,
                   std::string(), false);
  else
  {
    // updateStepPlan(resp.step_plan, msgs::UpdateMode::UPDATE_MODE_COST, std::string(), false);
    /// store planner statistics
    resp.statistics.expanded_states = planner_environment_->getNumExpandedStates();
    resp.statistics.visited_states = planner_environment_->getNumVisitedStates();

    resp.statistics.generated_footholds = StateSpaceManager::getNumFootholds();
    resp.statistics.generated_states = StateSpaceManager::getNumStates();
    resp.statistics.generated_planning_states = StateSpaceManager::getNumPlanningStates();

    resp.statistics.time_until_first_solution = planner_->get_initial_eps_planning_time();
    resp.statistics.planning_time = planner_->get_final_eps_planning_time();
    resp.statistics.final_eps = planner_->get_final_epsilon();

    /// store database statistics
    resp.statistics.foothold_hits = StateSpaceManager::footholdHits();
    resp.statistics.foothold_miss = StateSpaceManager::footholdMiss();
    resp.statistics.state_hits = StateSpaceManager::stateHits();
    resp.statistics.state_miss = StateSpaceManager::stateMiss();
    resp.statistics.planning_state_hits = StateSpaceManager::planningStatesHits();
    resp.statistics.planning_state_miss = StateSpaceManager::planningStatesMiss();

    if (resp.status.error == msgs::ErrorStatus::NO_ERROR && resp.statistics.final_eps > 1.8)
      resp.status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlanner",
                                        "stepPlanRequestService: Suboptimal plan (eps: " + boost::lexical_cast<std::string>(resp.statistics.final_eps) + ")!");
  }

  // some debug outputs and visualization stuff
  //  double total_cost = 0.0;
  //  double last_cost = 0.0;

  if (print_solution_)
  {
    for (const msgs::Step& step : resp.step_plan.plan.steps)
    {
      ROS_INFO("-------------------------------------");
      for (const msgs::FootStepData& foot_step : step.foot_steps)
      {
        geometry_msgs::Vector3 n;
        quaternionToNormal(foot_step.target.pose.orientation, n);
        ROS_INFO("[%i][%i] x/y/z - yaw: %.3f/%.3f/%.3f - %.3f", step.idx, foot_step.target.idx, foot_step.target.pose.position.x, foot_step.target.pose.position.y,
                 foot_step.target.pose.position.z, tf::getYaw(foot_step.target.pose.orientation));
        ROS_INFO("[%i][%i] n: %.3f/%.3f/%.3f", step.idx, foot_step.target.idx, n.x, n.y, n.z);
        ROS_INFO("[%i][%i] swing height: %.3f, sway duration: %.3f, step duration: %.3f", step.idx, foot_step.target.idx, foot_step.swing_height, foot_step.sway_duration,
                 foot_step.step_duration);
        // ROS_INFO("[%i][%i] valid: %s, colliding: %s", step.idx, step_data.target.idx, step.valid ? "y" : "n", step.colliding ? "y" : "n");
        // ROS_INFO("[%i][%i] cost: %.3f risk: %.3f", step.idx, step_data.target.idx, step.cost, step.risk);

        // total_cost += step.cost;
        // last_cost = step.cost;

        ROS_INFO("-------------------------------------");
      }

      for (const msgs::Foothold& foothold : step.support_feet)
      {
        geometry_msgs::Vector3 n;
        quaternionToNormal(foothold.pose.orientation, n);
        ROS_INFO("[%i][%i] > x/y/z - yaw: %.3f/%.3f/%.3f - %.3f (Support)", step.idx, foothold.idx, foothold.pose.position.x, foothold.pose.position.y, foothold.pose.position.z,
                 tf::getYaw(foothold.pose.orientation));
        ROS_INFO("[%i][%i] > n: %.3f/%.3f/%.3f (Support)", step.idx, foothold.idx, n.x, n.y, n.z);

        ROS_INFO("-------------------------------------");
      }

      ROS_INFO(" ");
    }
    // ROS_INFO("Total path cost: %f (%f)", total_cost, total_cost-last_cost);
  }

  // transform step plan
  FootPoseTransformer::transformToRobotFrame(resp.step_plan);

  return true;
}

msgs::ErrorStatus FootstepPlanner::stepPlanRequest(msgs::StepPlanRequestService::Request& req, ResultCB result_cb, FeedbackCB feedback_cb, PreemptCB preempt_cb)
{
  // preempt any planning
  preemptPlanning();

  result_cb_ = result_cb;
  preempt_cb_ = preempt_cb;

  // load parameter set, if given
  if (!req.plan_request.parameter_set_name.data.empty())
  {
    if (!ParameterManager::setActive(req.plan_request.parameter_set_name.data))
      return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_PARAMETERS, "FootstepPlanner",
                              "stepPlanRequest: Can't find parameter set named '" + req.plan_request.parameter_set_name.data + "'!");

    setParams(ParameterManager::getActive());
  }
  else
  {
    // just reset planner
    reset();
  }

  // transform feet poses
  FootPoseTransformer::transformToPlannerFrame(req.plan_request.start_footholds);
  FootPoseTransformer::transformToPlannerFrame(req.plan_request.goal_footholds);

  // set request specific parameters
  start_foot_idx_ = req.plan_request.start_foot_idx;
  feedback_handler_.setFrameId(req.plan_request.header.frame_id);
  feedback_handler_.setFeedbackCB(feedback_cb);

  // check if gait generator is available
  if (!RobotModel::gaitGenerator())
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner", "stepPlanRequest: No gait generator available!");

  // prepare planning
  msgs::ErrorStatus status = preparePlanning(req);

  if (!hasError(status))
  {
    // start planning in seperate thread
    planning_thread_ = boost::thread(&FootstepPlanner::doPlanning, this, req);

    // transform feet poses back
    FootPoseTransformer::transformToRobotFrame(req.plan_request.start_footholds);
    FootPoseTransformer::transformToRobotFrame(req.plan_request.goal_footholds);
  }

  return status;
}

bool FootstepPlanner::stepPlanRequestService(msgs::StepPlanRequestService::Request& req, msgs::StepPlanRequestService::Response& resp)
{
  // generate step plan based on request
  resp.status += stepPlanRequest(req);

  if (hasError(resp.status))
    return true;  // return always true so the message is returned

  // wait for thread to terminate
  planning_thread_.join();

  // finalize plan and generate response
  finalizeStepPlan(req, resp, true);

  return true;  // return always true so the message is returned
}

msgs::ErrorStatus FootstepPlanner::preparePlanning(msgs::StepPlanRequestService::Request& req)
{
  // set world model mode
  if (req.plan_request.planning_mode == msgs::StepPlanRequest::PLANNING_MODE_3D)
    WorldModel::mutableInstance().useTerrainModel(true);
  else if (req.plan_request.planning_mode == msgs::StepPlanRequest::PLANNING_MODE_PATTERN)
    WorldModel::mutableInstance().useTerrainModel(req.plan_request.pattern_parameters.use_terrain_model);
  else
    WorldModel::mutableInstance().useTerrainModel(false);

  // set start and goal
  if (req.plan_request.planning_mode != msgs::StepPlanRequest::PLANNING_MODE_PATTERN)
  {
    // set start foot poses
    if (!setStart(req.plan_request, true))  /// @TODO: Hack to disable collision check for start pose
      return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_START, "FootstepPlanner", "preparePlanning: Could not set start pose! Please check if poses are set!");

    // set goal foot poses
    if (!setGoal(req.plan_request))
      return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_GOAL, "FootstepPlanner", "preparePlanning: Could not set goal pose! Please check if poses are set!");
  }

  // call preparation step of plugins
  std::vector<FootstepPlanningPlugin::Ptr> plugins;
  vigir_pluginlib::PluginManager::getPlugins(plugins);
  for (FootstepPlanningPlugin::Ptr p : plugins)
    p->preparePlanning(req.plan_request);

  return msgs::ErrorStatus();
}

void FootstepPlanner::doPlanning(msgs::StepPlanRequestService::Request& req)
{
  // lock entire planning system
  boost::recursive_mutex::scoped_lock lock(planner_mutex_);
  // and world model
  WorldModel::ModelLocks model_locks(WorldModel::instance().lockModel());

  msgs::StepPlanRequestService::Response resp;

  // dispatch planning mode and plan
  switch (req.plan_request.planning_mode)
  {
    case msgs::StepPlanRequest::PLANNING_MODE_2D:
    case msgs::StepPlanRequest::PLANNING_MODE_3D:
      feedback_handler_.start(env_params_->feedback_rate);
      resp.status = planSteps(req);
      feedback_handler_.reset();
      break;
    case msgs::StepPlanRequest::PLANNING_MODE_PATTERN:
      goal_state_.reset();
      goal_pose_set_up_ = false;
      resp.status = planPattern(req);
      break;
    default:
      resp.status = ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner",
                                     "stepPlanRequest: A not supported planning mode '" + boost::lexical_cast<std::string>(req.plan_request.planning_mode) + "' was given!");
      break;
  }

  // result callbacks
  if (!result_cb_.empty())
  {
    // finalize plan and generate response
    if (!hasError(resp.status))
      finalizeStepPlan(req, resp, true);
    result_cb_(resp);
  }
}

void FootstepPlanner::preemptPlanning()
{
  if (!isPlanning())
    return;

  feedback_handler_.reset();

  // interrupt main planning thread
  planning_thread_.interrupt();
  planning_thread_.join();

  if (!preempt_cb_.empty())
    preempt_cb_();
}

/// @TODO: This shoud go into WorldModel
bool FootstepPlanner::findNearestValidFoothold(Foothold& foothold) const
{
  if (WorldModel::instance().isAccessible(foothold))
    return true;

  Foothold current_foothold = foothold;
  Foothold best_foothold = foothold;

  double pos_diff = FLT_MAX;
  double yaw_diff = FLT_MAX;
  bool solution_found = false;

  Point orig_pos;
  Point trans_pos;
  orig_pos.setZ(0.0);

  for (double yaw = -0.2; yaw <= 0.4; yaw += env_params_->res.toCont().angle)
  {
    /// @TODO: use RobotModelPlugin; assumes humanoid here
    current_foothold.setYaw(foothold.yaw() + (foothold.idx == 1 ? yaw : -yaw));
    for (double y = -0.05; y <= 0.2; y += env_params_->res.toCont().y)
    {
      /// @TODO: use RobotModelPlugin; assumes humanoid here
      orig_pos.setY(foothold.idx == 1 ? y : -y);
      for (double x = -0.15; x <= 0.15; x += env_params_->res.toCont().x)
      {
        // determine point in world frame and get height at this point
        orig_pos.setX(x);

        // get transformation foot -> world
        trans_pos = foothold.pose().inverse() * orig_pos;

        current_foothold.setX(trans_pos.x());
        current_foothold.setY(trans_pos.y());

        if (WorldModel::instance().update3DData(current_foothold) < TerrainResult::OK)
          continue;

        if (!WorldModel::instance().isAccessible(current_foothold))
          continue;

        double dist = std::sqrt(x * x + y * y);
        if (pos_diff >= dist && yaw_diff >= std::abs(yaw))
        {
          best_foothold = current_foothold;
          pos_diff = dist;
          yaw_diff = std::abs(yaw);
          solution_found = true;
        }
      }
    }
  }

  if (solution_found)
    foothold = best_foothold;

  return solution_found;
}

bool FootstepPlanner::setStart(const State& state, bool ignore_collision)
{
  // check for errors
  for (Foothold::ConstPtr f : state.getFootholds())
  {
    ROS_ASSERT(!std::isnan(f->roll()));
    ROS_ASSERT(!std::isnan(f->pitch()));
    ROS_ASSERT(!std::isnan(f->yaw()));
  }

  // check reachability
  if (!Reachability::instance().isReachable(state))
  {
    ROS_ERROR("[FootstepPlanner] Start state not reachable! Check input start or robot model consistency!");
    return false;
  }

  // collision check
  if (!ignore_collision && !WorldModel::instance().isAccessible(state))
  {
    ROS_ERROR("[FootstepPlanner] Start state not accessible!");
    return false;
  }

  start_state_ = StateSpaceManager::addState(state);
  start_pose_set_up_ = true;

  return true;
}

bool FootstepPlanner::setStart(msgs::StepPlanRequest& req, bool ignore_collision)
{
  FootholdArray footholds;
  footholdArrayMsgToL3(req.start_footholds, footholds);

  FloatingBaseArray floating_bases;
  floatingBaseArrayMsgToL3(req.start_floating_bases, floating_bases);

  if (footholds.empty() && floating_bases.empty())
  {
    ROS_ERROR("[FootstepPlanner] Start state was empty!");
    return false;
  }

  // post process footholds
  for (Foothold& f : footholds)
  {
    if (!PostProcessor::instance().postProcess(f))
    {
      ROS_ERROR("[FootstepPlanner] Start state post processing failed!");
      return false;
    }
  }

  // generate state
  State s(StateSpaceManager::addFootholds(footholds), FloatingBaseHashedConstPtrArray(StateSpaceManager::addFloatingBases(floating_bases)));
  s.setIsStart(true);

  if (!PostProcessor::instance().postProcess(s))
  {
    ROS_ERROR("[FootstepPlanner] Start state post processing failed!");
    return false;
  }

  // if state post processing did not generate any floating bases, then use the given from request
  if (!s.hasFloatingBases())
  {
    // post process floating base
    for (FloatingBase& fb : floating_bases)
    {
      if (!PostProcessor::instance().postProcess(fb))
      {
        ROS_ERROR("[FootstepPlanner] Start state floating base post processing failed!");
        return false;
      }
      s.updateFloatingBase(StateSpaceManager::addFloatingBase(fb));
    }
  }

  // return actually used state
  footholdArrayL3ToMsg(s.getFootholds(), req.start_footholds);
  floatingBaseArrayL3ToMsg(s.getFloatingBases(), req.start_floating_bases);

  return setStart(s, ignore_collision);
}

bool FootstepPlanner::setGoal(const State& state, bool ignore_collision)
{
  // check for errors
  for (Foothold::ConstPtr f : state.getFootholds())
  {
    ROS_ASSERT(!std::isnan(f->roll()));
    ROS_ASSERT(!std::isnan(f->pitch()));
    ROS_ASSERT(!std::isnan(f->yaw()));
  }

  // check reachability
  if (!Reachability::instance().isReachable(state))
  {
    ROS_ERROR("[FootstepPlanner] Goal state not reachable! Check input goal or robot model consistency!");
    return false;
  }

  // collision check
  if (!ignore_collision && !WorldModel::instance().isAccessible(state))
  {
    ROS_ERROR("[FootstepPlanner] Goal state not accessible!");
    return false;
  }

  goal_state_ = StateSpaceManager::addState(state);
  goal_pose_set_up_ = true;

  return true;
}

bool FootstepPlanner::setGoal(msgs::StepPlanRequest& req, bool ignore_collision)
{
  FootholdArray footholds;
  footholdArrayMsgToL3(req.goal_footholds, footholds);

  FloatingBaseArray floating_bases;
  floatingBaseArrayMsgToL3(req.goal_floating_bases, floating_bases);

  if (footholds.empty() && floating_bases.empty())
  {
    ROS_ERROR("[FootstepPlanner] Goal state was empty!");
    return false;
  }

  // post process footholds
  for (Foothold& f : footholds)
  {
    if (!PostProcessor::instance().postProcess(f))
    {
      ROS_ERROR("[FootstepPlanner] Goal state post processing failed!");
      return false;
    }
  }

  // generate state
  State s(StateSpaceManager::addFootholds(footholds), StateSpaceManager::addFloatingBases(floating_bases));
  s.setIsGoal(true);

  if (!PostProcessor::instance().postProcess(s))
  {
    ROS_ERROR("[FootstepPlanner] Goal state post processing failed!");
    return false;
  }

  // if state post processing did not generate any floating bases, then use the given from request
  if (!s.hasFloatingBases())
  {
    // post process floating base
    for (FloatingBase& fb : floating_bases)
    {
      if (!PostProcessor::instance().postProcess(fb))
      {
        ROS_ERROR("[FootstepPlanner] Goal state floating base post processing failed!");
        return false;
      }
      s.updateFloatingBase(StateSpaceManager::addFloatingBase(fb));
    }
  }

  // return actually used state
  footholdArrayL3ToMsg(s.getFootholds(), req.goal_footholds);
  floatingBaseArrayL3ToMsg(s.getFloatingBases(), req.goal_floating_bases);

  return setGoal(s, ignore_collision);
}
}  // namespace l3_footstep_planning
