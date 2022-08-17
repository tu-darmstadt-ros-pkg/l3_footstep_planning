#include <l3_footstep_planning_vis_tools/std/step_plan_path_vis.h>

#include <l3_libs/conversions/l3_msg_conversions.h>
#include <l3_libs/helper.h>

namespace l3_footstep_planning
{
StepPlanPathVis::StepPlanPathVis()
  : PlanningVisPlugin("step_plan_path_vis")
{}

bool StepPlanPathVis::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PlanningVisPlugin::initialize(params))
    return false;

  std::string topic = param("topic", std::string("vis/step_plan_path"), true);
  step_plan_path_pub_ = nh_.advertise<nav_msgs::Path>(topic, 1, true);

  return true;
}

void StepPlanPathVis::clear()
{
  path_.poses.clear();
  is_updated_ = true;
}

void StepPlanPathVis::visualize(const StepPlan& step_plan)
{
  stepPlanToPath(step_plan.getSteps().asArray<StepConstPtrArray>(), path_);
  is_updated_ = true;
}

void StepPlanPathVis::trigger()
{
  if (is_updated_ && !path_.header.frame_id.empty())
  {
    step_plan_path_pub_.publish(path_);
    is_updated_ = false;
  }
}

void StepPlanPathVis::stepPlanToPath(StepConstPtrArray steps, nav_msgs::Path& path) const
{
  path.poses.clear();

  if (steps.empty())
    return;

  /// @TODO: Add consisty checks

  for (Step::ConstPtr step : steps)
  {
    FootholdConstPtrArray footholds;

    if (step->hasStepData())
    {
      for (FootStepData::ConstPtr foot_step : valuesAsArray<FootStepDataConstPtrArray>(step->getStepDataMap()))
        footholds.push_back(foot_step->target);
    }
    else
    {
      footholds = valuesAsArray<FootholdConstPtrArray>(step->getSupportFootMap());
    }

    for (Foothold::ConstPtr f : footholds)
    {
      if (!ignoreFootIdx(f->idx))
      {
        geometry_msgs::PoseStamped pose;
        pose.header = f->header;
        poseL3ToMsg(f->pose(), pose.pose);
        path.poses.push_back(pose);
      }
    }
  }

  if (!path.poses.empty())
    path.header = path.poses.front().header;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::StepPlanPathVis, l3_footstep_planning::PlanningVisPlugin)
