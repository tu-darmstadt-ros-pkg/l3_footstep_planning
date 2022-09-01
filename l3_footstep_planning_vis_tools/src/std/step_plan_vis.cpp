#include <l3_footstep_planning_vis_tools/std/step_plan_vis.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_vis_tools/visualization.h>

namespace l3_footstep_planning
{
StepPlanVis::StepPlanVis()
  : PlanningMarkersVisPlugin("step_plan_vis")
{
  marker_ns_ = "step_plan";
}

bool StepPlanVis::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PlanningMarkersVisPlugin::loadParams(params))
    return false;

  // get parameters
  add_step_index_ = param("add_step_index", true, true);

  return true;
}

void StepPlanVis::visualize(const StepPlan& step_plan)
{
  msgs::StepArray steps;
  stepArrayL3ToMsg(step_plan.getSteps().asArray(), steps);
  markers_ = stepPlanToFootMarkerArray(steps, *RobotModel::description(), add_step_index_, marker_ns_);
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::StepPlanVis, l3_footstep_planning::PlanningVisPlugin)
