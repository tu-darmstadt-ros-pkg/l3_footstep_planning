#include <l3_footstep_planning_vis_tools/std/upper_body_vis.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_vis_tools/visualization.h>

namespace l3_footstep_planning
{
UpperBodyVis::UpperBodyVis()
  : PlanningMarkersVisPlugin("upper_body_vis")
{
  marker_ns_ = "upper_body";
}

void UpperBodyVis::visualize(const StepPlan& step_plan)
{
  msgs::FootholdArray start;
  footholdArrayL3ToMsg(step_plan.getStart(), start);

  msgs::StepArray steps;
  stepArrayL3ToMsg(step_plan.getSteps().asArray(), steps);

  markers_ = stepPlanToUpperBodyMarkerArray(start, steps, *RobotModel::description(), true, marker_ns_);
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::UpperBodyVis, l3_footstep_planning::PlanningVisPlugin)
