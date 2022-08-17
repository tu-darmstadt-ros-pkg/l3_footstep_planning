#include <l3_footstep_planner/pattern_planner.h>

#include <l3_libs/types/types.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_plugins/aggregator/reachability.h>
#include <l3_footstep_planning_plugins/aggregator/world_model.h>
#include <l3_footstep_planning_plugins/aggregator/post_processor.h>

namespace l3_footstep_planning
{
PatternPlanner::PatternPlanner(const DiscreteResolution& res)
  : res_(res)
  , fake_uid_(0)
{}

void PatternPlanner::reset()
{
  footsteps_.clear();
  plan_.clear();
  fake_uid_ = 0;
}

msgs::ErrorStatus PatternPlanner::planPattern(StepPtrArray& path, StateHashed::ConstPtr start_state, const msgs::StepPlanRequestService::Request& req)
{
  msgs::ErrorStatus status;

  reset();

  const msgs::PatternParameters& params = req.plan_request.pattern_parameters;

  unsigned int num_steps = params.steps;
  bool close_step = params.close_step && num_steps > 0;
  single_step_mode_ = false;

  change_z_ = true;

  // determine collision free start uid
  fake_uid_ = std::max(fake_uid_, start_state->getUID());
  for (FootholdHashed::ConstPtr f : start_state->getFootholdsHashed())
    fake_uid_ = std::max(fake_uid_, f->getUID());
  fake_uid_++;

  ROS_INFO("Start planning stepping (mode: %u, steps: %u)\n", params.mode, num_steps);

  // prepare start state
  pstate_.reset(new PlanningState(start_state));
  pstate_->getStep()->setStepIndex(0);

  // add start state
  plan_.push_back(pstate_);

  // configure pattern
  switch (params.mode)
  {
    case msgs::PatternParameters::FEET_REALIGN:
    {
      status += generateNeutralStancePattern(params, true);
      break;
    }
    case msgs::PatternParameters::WIDE_STANCE:
    {
      /// @TODO: Add larger foot seperation
      status += generateNeutralStancePattern(params, true);
      break;
    }
    default:
    {
      status += configureGenerator(params);
      if (hasError(status))
        return status;

      // generate common used step placements
      //  Footstep step_up(0, 1.4*env_params_->foot_size.x, p.second.neutral_stance.y(), 0.0, 0.0, cell_size_, num_angle_bins_);
      //  Footstep step_down(0, 1.1*env_params_->foot_size.x, p.second.neutral_stance.y(), 0.0, 0.0, cell_size_, num_angle_bins_);

      // add step up motion
      if (params.mode == msgs::PatternParameters::STEP_UP || params.mode == msgs::PatternParameters::STEP_OVER)
      {
        /// @TODO
        double step_up_height = std::abs(params.dz);
        status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "PatternPlanner", "Step over pattern is not supported yet.");
      }

      // add pattern
      status += generateWalkPattern(params, req.plan_request.start_foot_idx);
      if (hasError(status))
        return status;

      // add step down motion
      if (params.mode == msgs::PatternParameters::STEP_DOWN || params.mode == msgs::PatternParameters::STEP_OVER)
      {
        /// @TODO
      }

      // add final steps so feet are parallel
      if (close_step)
        status += generateNeutralStancePattern(params);
    }
  }

  if (hasError(status))
    return status;

  // start state consists of all "non-moving step data" of all feet
  for (Foothold::ConstPtr f : start_state->getFootholds())
    plan_.front()->getStep()->updateSupportFoot(f);

  // finally extract path
  status += extractPath(path);

  return status;
}

msgs::ErrorStatus PatternPlanner::configureGenerator(const l3_footstep_planning_msgs::PatternParameters& params)
{
  single_step_mode_ = false;

  switch (params.mode)
  {
    case msgs::PatternParameters::STEP_UP:
    case msgs::PatternParameters::STEP_DOWN:
    case msgs::PatternParameters::STEP_OVER:
      change_z_ = false;  // as dz is used for step up/down motion
    case msgs::PatternParameters::FORWARD:
    {
      footsteps_ = generateFootsteps(params.step_distance_forward, 0.0, 0.0);
      break;
    }
    case msgs::PatternParameters::BACKWARD:
    {
      footsteps_ = generateFootsteps(-params.step_distance_forward, 0.0, 0.0);
      break;
    }
    case msgs::PatternParameters::STRAFE_LEFT:
    {
      footsteps_ = generateFootsteps(0.0, params.step_distance_sideward, 0.0);
      single_step_mode_ = true;
      break;
    }
    case msgs::PatternParameters::STRAFE_RIGHT:
    {
      footsteps_ = generateFootsteps(0.0, -params.step_distance_sideward, 0.0);
      single_step_mode_ = true;
      break;
    }
    case msgs::PatternParameters::ROTATE_LEFT:
    {
      footsteps_ = generateFootsteps(0.0, 0.0, params.turn_angle);
      single_step_mode_ = true;
      break;
    }
    case msgs::PatternParameters::ROTATE_RIGHT:
    {
      footsteps_ = generateFootsteps(0.0, 0.0, -params.turn_angle);
      single_step_mode_ = true;
      break;
    }
    case msgs::PatternParameters::SAMPLING:
    {
      footsteps_ = generateFootsteps(params.step_distance_forward, params.step_distance_sideward, params.turn_angle);
      break;
    }
    default:
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner",
                              "PatternPlanner: Unknown walk mode (" + boost::lexical_cast<std::string>((int)params.mode) + ") set!");
  }

  return msgs::ErrorStatus();
}

msgs::ErrorStatus PatternPlanner::generateWalkPattern(const l3_footstep_planning_msgs::PatternParameters& params, const FootIndex& start_foot_idx)
{
  msgs::ErrorStatus status;

  for (unsigned int i = 0; i < params.steps; i++)
  {
    const State& current_state = *pstate_->getState();
    //const FootIndexArray& last_seq = pstate_->getMovedFootIndeces();
    ExpandStatesIdx last_seq;
    last_seq.foot_idx = pstate_->getMovedFootIndeces();
    const ExpandStatesIdxArray& patterns = RobotModel::gaitGenerator()->succMovingPatterns(pstate_->getStep(), { last_seq });

    if (patterns.empty())
    {
      status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "PatternPlanner", "GaitGenerator did not provide any patterns!");
      return status;
    }

    // trying all patterns until one succeeds (assumes prioritized list)
    bool success = false;
    for (const ExpandStatesIdx& pattern : patterns)
    {
      // handle start foot selection
      if (i == 0 && start_foot_idx != msgs::StepPlanRequest::AUTO_START_FOOT_IDX)
      {
        bool valid = false;
        for (const FootIndex& idx : pattern.foot_idx)
        {
          if (idx == start_foot_idx)
          {
            valid = true;
            break;
          }
        }

        if (!valid)
          continue;
      }

      // generate next step sequence
      State new_state(current_state);

      for (const FootIndex& idx : pattern.foot_idx)
      {
        const FootStep& footstep = footsteps_.find(idx)->second;
        Foothold::Ptr foothold = footstep.getSuccessor(current_state);

        if (params.override)
        {
          foothold->setRoll(params.roll);
          foothold->setPitch(params.pitch);
        }
        else if (params.use_terrain_model)
          WorldModel::instance().update3DData(*foothold);
        else if (change_z_)
          foothold->setZ(current_state.getFoothold(foothold->idx)->z() + params.dz);

        new_state.updateFoothold(FootholdHashed::make(*foothold, fake_uid_++));
      }

      PlanningState::Ptr next_pstate(new PlanningState(StateHashed::make(new_state, fake_uid_++), pstate_->getState()));

      // post processing and model checking
      PostProcessor::instance().postProcess(*next_pstate);

      if (!params.ignore_robot_model && !Reachability::instance().isReachable(*next_pstate))
        continue;

      if (params.use_terrain_model && !WorldModel::instance().isAccessible(*next_pstate))
        continue;

      // push step into path
      next_pstate->getStep()->setStepIndex(pstate_->getStep()->getStepIndex() + 1);

      pstate_ = next_pstate;
      success = true;
      break;
    }

    if (!success)
    {
      status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "PatternPlanner", "Could not generate valid successor! Check reachability and world model.");
      return status;
    }

    // add next step to plan
    plan_.push_back(pstate_);

    if (hasError(status))
      return status;

    // in single step mode, the robot must return in neutral stance before the next step sequence
    if (single_step_mode_ && (!params.close_step || i < (params.steps - 1)))
    {
      status += generateNeutralStancePattern(params);
      if (hasError(status))
        return status;
    }
  }

  return status;
}

msgs::ErrorStatus PatternPlanner::generateNeutralStancePattern(const l3_footstep_planning_msgs::PatternParameters& params, bool force_all)
{
  msgs::ErrorStatus status;

  // determine feet which are not in final position
  FootholdPtrArray neutral_stance;
  if (pstate_->getStep()->hasStepData())
    neutral_stance = RobotModel::description()->getNeutralStance(*pstate_->getStep()->getStepDataMap().begin()->second->target);
  else
    neutral_stance = RobotModel::description()->getNeutralStance(pstate_->getFeetCenter());

  std::map<FootIndex, Foothold> open_footholds;

  for (Foothold::ConstPtr f : neutral_stance)
  {
    if (force_all || *f != *pstate_->getState()->getFoothold(f->idx))
      open_footholds[f->idx] = *f;
  }

  while (!open_footholds.empty())
  {
    const State& current_state = *pstate_->getState();
    //const FootIndexArray& last_seq = pstate_->getMovedFootIndeces();
    ExpandStatesIdx last_seq;
    last_seq.foot_idx = pstate_->getMovedFootIndeces();
    const ExpandStatesIdxArray& patterns = RobotModel::gaitGenerator()->succMovingPatterns(pstate_->getStep(), { last_seq });

    if (patterns.empty())
    {
      status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "PatternPlanner", "GaitGenerator did not provide any patterns!");
      return status;
    }

    // trying all patterns until one succeeds (assumes prioritized list)
    bool success = false;
    for (const ExpandStatesIdx& a : patterns)
    {
      // generate next step sequence
      State new_state(current_state);

      bool updated = false;
      for (const FootIndex& idx : a.foot_idx)
      {
        FootholdHashedConstPtrArray footholds_ptr;

        std::map<FootIndex, Foothold>::const_iterator itr = open_footholds.find(idx);
        if (itr != open_footholds.end())
        {
          Foothold foothold = itr->second;

          if (params.override)
          {
            foothold.setRoll(params.roll);
            foothold.setPitch(params.pitch);
          }
          else if (params.use_terrain_model)
            WorldModel::instance().update3DData(foothold);

          footholds_ptr.push_back(FootholdHashed::make(foothold, fake_uid_++));
          open_footholds.erase(itr);
          updated = true;
        }

        new_state.updateFootholds(footholds_ptr);
      }

      if (!updated)
        continue;

      // post processing and model checking
      // WARNING: Using move operation here; do not use new_state object from here
      PlanningState::Ptr next_pstate(new PlanningState(StateHashed::make(std::move(new_state), fake_uid_++), pstate_->getState()));

      PostProcessor::instance().postProcess(*next_pstate);

      if (!params.ignore_robot_model && !Reachability::instance().isReachable(*next_pstate))
        continue;

      if (params.use_terrain_model && !WorldModel::instance().isAccessible(*next_pstate))
        continue;

      // push step into path
      next_pstate->getStep()->setStepIndex(pstate_->getStep()->getStepIndex() + 1);

      pstate_ = next_pstate;
      success = true;
      break;
    }

    if (!success)
    {
      status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "PatternPlanner", "Could not generate valid state to reach neutral stance! Check reachability and world model.");
      return status;
    }

    // add next step to plan
    plan_.push_back(pstate_);

    if (hasError(status))
      return status;
  }

  return status;
}

msgs::ErrorStatus PatternPlanner::extractPath(StepPtrArray& path)
{
  path.clear();
  for (PlanningState::Ptr s : plan_)
    path.push_back(s->getStep());

  return msgs::ErrorStatus();
}

std::map<FootIndex, FootStep> PatternPlanner::generateFootsteps(double dx, double dy, double dyaw) const
{
  std::map<FootIndex, FootStep> result;

  RobotDescription::ConstPtr description = RobotModel::instance().description();
  for (Foothold::ConstPtr f : description->getNeutralStance(Pose(dx, dy, 0.0, 0.0, 0.0, dyaw)))
  {
    const Pose& n = description->getFootInfo(f->idx).neutral_stance;
    result.emplace(f->idx, FootStep(n, f->idx, f->x() - n.x(), f->y() - n.y(), f->yaw() - n.yaw(), 0.0, res_));
  }

  return result;
}

Foothold PatternPlanner::getFootPose(const Pose& robot, const FootIndex& foot_idx, double dx, double dy, double dyaw) const
{
  RobotDescription::ConstPtr description = RobotModel::instance().description();

  Pose pose(robot.x() + dx, robot.y() + dy, robot.z(), robot.roll(), robot.pitch(), robot.yaw() + dyaw);
  for (Foothold::ConstPtr f : description->getNeutralStance(pose))
  {
    if (f->idx == foot_idx)
      return *f;
  }

  return Foothold();
}
}  // namespace l3_footstep_planning
