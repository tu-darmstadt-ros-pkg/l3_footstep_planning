#include <l3_footstep_planning_vis_tools/std/step_plan_request_vis.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_vis_tools/visualization.h>

namespace l3_footstep_planning
{
StepPlanRequestVis::StepPlanRequestVis()
  : PlanningMarkersVisPlugin("step_plan_request_vis")
{}

void StepPlanRequestVis::clear()
{
  markers_.markers.clear();
  markers_.markers.push_back(createResetMarker("start"));
  markers_.markers.push_back(createResetMarker("goal"));
}

void StepPlanRequestVis::visualize(const msgs::StepPlanRequest& step_plan_request)
{
  // generate start marker
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 0.5;
  color.b = 0.0;
  color.a = 1.0;
  markers_ = feetToFootMarkerArray(applyFootIdxWhitelist(step_plan_request.start_footholds), *RobotModel::description(), color, "start");

  /// @todo Implement multi floating base
  int floating_base_id = 0;
  if (!step_plan_request.start_floating_bases.empty())
  {
    visualization_msgs::Marker marker = baseToBaseMarker(step_plan_request.start_floating_bases[BaseInfo::MAIN_BODY_IDX], *RobotModel::description(), color, "start_base");
    floating_base_id = static_cast<int>(markers_.markers.size());
    marker.id = floating_base_id;

    marker.scale.x = BASE_END_POSE_SIZE;
    marker.scale.y = BASE_END_POSE_SIZE;
    marker.scale.z = BASE_END_POSE_SIZE;

    markers_.markers.push_back(marker);
  }

  // generate goal marker
  color.r = 0.0;
  color.g = 0.5;
  color.b = 1.0;
  color.a = 1.0;
  l3::appendMarkerArray(markers_, feetToFootMarkerArray(applyFootIdxWhitelist(step_plan_request.goal_footholds), *RobotModel::description(), color, "goal"));

  if (!step_plan_request.goal_floating_bases.empty())
  {
    visualization_msgs::Marker marker = baseToBaseMarker(step_plan_request.goal_floating_bases[BaseInfo::MAIN_BODY_IDX], *RobotModel::description(), color, "goal_base");
    marker.id = floating_base_id;

    marker.scale.x = 0.07;
    marker.scale.y = 0.07;
    marker.scale.z = 0.07;

    markers_.markers.push_back(marker);
  }
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::StepPlanRequestVis, l3_footstep_planning::PlanningVisPlugin)
