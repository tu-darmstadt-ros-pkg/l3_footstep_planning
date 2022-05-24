#include <l3_footstep_planning_plugins/aggregator/post_processor.h>

namespace l3_footstep_planning
{
PostProcessor::PostProcessor()
  : ExtendedPluginAggregator<PostProcessor, PostProcessPlugin>("PostProcessor")
{}

bool PostProcessor::postProcess(PlanningState& state) const
{
  bool result = true;
  for (PostProcessPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);
    result &= plugin->postProcess(state);
  }
  return result;
}

bool PostProcessor::postProcess(State& state, const FootIndexArray& updated_ids) const
{
  bool result = true;
  for (PostProcessPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);
    result &= plugin->postProcess(state, updated_ids);
  }
  return result;
}

bool PostProcessor::postProcess(Foothold& foothold) const
{
  bool result = true;
  for (PostProcessPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);
    result &= plugin->postProcess(foothold);
  }
  return result;
}

bool PostProcessor::postProcess(FloatingBase& floating_base) const
{
  bool result = true;
  for (PostProcessPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);
    result &= plugin->postProcess(floating_base);
  }
  return result;
}

bool PostProcessor::postProcess(Step& step) const
{
  bool result = true;
  for (PostProcessPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);
    result &= plugin->postProcess(step);
  }
  return result;
}

bool PostProcessor::postProcess(StepPlan& step_plan) const
{
  bool result = true;
  for (PostProcessPlugin::Ptr plugin : getPlugins())
  {
    ROS_ASSERT(plugin);
    result &= plugin->postProcess(step_plan);
  }
  return result;
}
}  // namespace l3_footstep_planning
