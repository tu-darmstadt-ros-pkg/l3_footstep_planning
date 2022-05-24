#include <l3_footstep_planning_plugins/aggregator/state_generator.h>
#include <l3_footstep_planning_plugins/aggregator/use_mask_generator.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_libs/helper.h>
#include <l3_footstep_planning_libs/modeling/state_space_manager.h>

namespace l3_footstep_planning
{
StateGenerator::StateGenerator()
  : ExtendedPluginAggregator<StateGenerator, StateGeneratorPlugin>("StateGenerator")
  , random_device_()
  , random_generator_(random_device_())
{}

bool StateGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!ExtendedPluginAggregator<StateGenerator, StateGeneratorPlugin>::loadParams(params))
    return false;

  int threads = params.param("threads", -1, true);
  unsigned int jobs_per_thread = params.param("jobs_per_thread", static_cast<unsigned int>(50), true);

  // setup state expansion manager
  expand_states_manager_.reset(new threading::ThreadingManager<threading::ExpandStateJob>(threads, jobs_per_thread));

  // load indirect foot idx list
  indirect_foot_idx_.clear();
  for (const FootInfoPair& p : RobotModel::description()->getFootInfoMap())
  {
    if (p.second.indirect)
      indirect_foot_idx_.insert(p.first);
  }

  return true;
}

std::list<PlanningStateGenResult> StateGenerator::generateNearStartStates(PlanningState::ConstPtr current, PlanningState::ConstPtr start, PlanningState::ConstPtr goal) const
{
  // ask for valid movement patterns
  ExpandStatesIdxArray next_seq({ ExpandStatesIdx(removeIndirectIdx(current->getMovedFootIndeces()), current->getMovedFloatingBaseIndeces()) });
  ExpandStatesIdxArray patterns = RobotModel::gaitGenerator()->predMovingPatterns(current->getStep(), next_seq);

  // generate new states
  return processJobs(generateNearStateJobs(current, start, start->getState(), goal->getState(), patterns));
}

std::list<PlanningStateGenResult> StateGenerator::generateNearGoalStates(PlanningState::ConstPtr current, PlanningState::ConstPtr start, PlanningState::ConstPtr goal) const
{
  // ask for valid movement patterns
  ExpandStatesIdxArray last_seq({ ExpandStatesIdx(removeIndirectIdx(current->getMovedFootIndeces()), current->getMovedFloatingBaseIndeces()) });
  ExpandStatesIdxArray patterns = RobotModel::gaitGenerator()->succMovingPatterns(current->getStep(), last_seq);

  // generate new states
  return processJobs(generateNearStateJobs(current, goal, start->getState(), goal->getState(), patterns));
}

std::vector<threading::ExpandStateJob::Ptr> StateGenerator::generatePredecessorsJobs(PlanningState::ConstPtr state, State::ConstPtr start, State::ConstPtr goal, bool lazy) const
{
  ExpandStatesIdx next_seq;

  if (state->getSuccState() && state->getStep())
  {
    next_seq.foot_idx = removeIndirectIdx(state->getMovedFootIndeces());
    next_seq.floating_base_idx = state->getMovedFloatingBaseIndeces();
  }
  else
  {
    /// @TODO: Use starting leg here
  }

  // get allowed patterns from gait generator
  ExpandStatesIdxArray patterns = RobotModel::gaitGenerator()->predMovingPatterns(state->getStep(), { next_seq });

  // get use mask
  UseMask mask = UseMaskGenerator::instance().determineStateGenerationUseMask(*state, *start, *goal);

  // generate job list based on new foothold patterns
  std::vector<threading::ExpandStateJob::Ptr> jobs;
  for (StateGeneratorPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);

    if (plugin->canUse(mask))
    {
      for (const ExpandStatesIdx& state_expansion_idx : patterns)
      {
        for (const StateGenResult& state_gen_result : plugin->generatePredStateResults(*state, *start, *goal, state_expansion_idx))
          jobs.push_back(threading::ExpandStateJob::make(state, start, goal, state_gen_result.footholds, state_gen_result.floating_base, false, lazy));
      }
    }
  }

  return jobs;
}

std::vector<threading::ExpandStateJob::Ptr> StateGenerator::generateSuccessorsJobs(PlanningState::ConstPtr state, State::ConstPtr start, State::ConstPtr goal, bool lazy) const
{
  ExpandStatesIdx last_seq;

  if (state->getPredState() && state->getStep())
  {
    last_seq.foot_idx = removeIndirectIdx(state->getMovedFootIndeces());
    last_seq.floating_base_idx = state->getMovedFloatingBaseIndeces();
  }
  else
  {
    /// @TODO: Use starting leg here
  }

  // get allowed patterns from gait generator
  ExpandStatesIdxArray patterns = RobotModel::gaitGenerator()->succMovingPatterns(state->getStep(), { last_seq });

  // get use mask
  UseMask mask = UseMaskGenerator::instance().determineStateGenerationUseMask(*state, *start, *goal);

  // generate job list based on new foothold patterns
  std::vector<threading::ExpandStateJob::Ptr> jobs;
  for (StateGeneratorPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);

    if (plugin->canUse(mask))
    {
      for (const ExpandStatesIdx& state_expansion_idx : patterns)
      {
        for (const StateGenResult& state_gen_result : plugin->generateSuccStateResults(*state, *start, *goal, state_expansion_idx))
          jobs.push_back(threading::ExpandStateJob::make(state, start, goal, state_gen_result.footholds, state_gen_result.floating_base, true, lazy));
      }
    }
  }

  return jobs;
}

std::vector<threading::ExpandStateJob::Ptr> StateGenerator::selectRandomJobs(std::vector<threading::ExpandStateJob::Ptr> jobs, unsigned int num) const
{
  if (num < jobs.size())
  {
    std::uniform_int_distribution<> distribution(0, static_cast<int>(jobs.size()) - 1);

    std::vector<threading::ExpandStateJob::Ptr> rnd_jobs;
    rnd_jobs.reserve(num);

    // adding random elements until desired number of jobs is reached
    while (rnd_jobs.size() < num)
    {
      unsigned long x = static_cast<unsigned long>(distribution(random_generator_));

      ROS_ASSERT(x < jobs.size());

      // moving randomly selected job entry
      threading::ExpandStateJob::Ptr& job = jobs[x];
      if (job)
      {
        rnd_jobs.push_back(job);
        job.reset();
      }
    }

    return rnd_jobs;
  }
  else
    return jobs;
}

std::vector<threading::ExpandStateJob::Ptr> StateGenerator::generateNearStateJobs(PlanningState::ConstPtr current, PlanningState::ConstPtr target, State::ConstPtr start,
                                                                                  State::ConstPtr goal, const ExpandStatesIdxArray& patterns) const
{
  if (patterns.empty())
  {
    ROS_ERROR("generateNearStates: No patterns given. Fix it!");
    return std::vector<threading::ExpandStateJob::Ptr>();
  }

  // get use mask
  UseMask mask = UseMaskGenerator::instance().determineStateGenerationUseMask(*current, *start, *goal);

  // generate job list based on new foothold patterns
  std::vector<threading::ExpandStateJob::Ptr> jobs;
  for (StateGeneratorPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);

    if (plugin->canUse(mask))
    {
      for (const ExpandStatesIdx& state_expansion_idx : patterns)
      {
        for (const StateGenResult& state_gen_result : plugin->generateNearStateResults(*current, *target, state_expansion_idx))
          jobs.push_back(threading::ExpandStateJob::make(current, start, goal, state_gen_result.footholds, state_gen_result.floating_base, true));
      }
    }
  }

  return jobs;
}

FootIndexArray StateGenerator::removeIndirectIdx(FootIndexArray foot_idx) const
{
  if (indirect_foot_idx_.empty())
    return foot_idx;
  else
  {
    FootIndexArray result;
    for (const FootIndex& idx : foot_idx)
    {
      if (indirect_foot_idx_.find(idx) == indirect_foot_idx_.end())
        result.push_back(idx);
    }
    return result;
  }
}
}  // namespace l3_footstep_planning
