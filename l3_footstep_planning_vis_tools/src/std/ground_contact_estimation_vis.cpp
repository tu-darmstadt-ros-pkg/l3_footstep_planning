#include <l3_footstep_planning_vis_tools/std/ground_contact_estimation_vis.h>

#include <l3_libs/conversions/l3_msg_step_conversions.h>

#include <l3_vis/visualization.h>

namespace l3_footstep_planning
{
GroundContactEstimationVis::GroundContactEstimationVis()
  : PlanningMarkersVisPlugin("ground_contact_estimation_vis")
{
  marker_ns_ = "ground_contact_estimation";
}

void GroundContactEstimationVis::visualize(const msgs::PlanningFeedback& planning_feedback)
{
  if (!planning_feedback.last_visited_step.foot_steps.empty())
  {
    Step step;
    stepMsgToL3(planning_feedback.last_visited_step, step);
    visualize(step);
  }
}

void GroundContactEstimationVis::visualize(const Step& step)
{
  if (!step.footStep().hasMovingLinks())
    return;

  visualization_msgs::Marker marker;
  marker.header = step.footStep().getMovingLinks().begin()->second->target->header;
  marker.ns = marker_ns_;
  marker.id = markers_.markers.size();
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.0;

  for (const Step::FootStep::MovingDataPair& p : step.footStep().getMovingLinks())
  {
    Foothold::ConstPtr foothold = p.second->target;

    Vector4Array checked_positions;
    if (foothold->data.get("gc_checked_positions", checked_positions))
    {
      for (const Vector4& v : checked_positions)
      {
        geometry_msgs::Point point;
        point.x = static_cast<float>(v.x());
        point.y = static_cast<float>(v.y());
        point.z = static_cast<float>(v.z());
        marker.points.push_back(point);

        // determine color
        std_msgs::ColorRGBA color;
        if (v.w() < 0.0)  // intrusion (green to red)
          color = getColorScale(createColorMsg(0.1, 0.8, 0.1), createColorMsg(0.8, 0.1, 0.1), std::abs(v.w()));
        else  // overhang (green to purple)
          color = getColorScale(createColorMsg(0.1, 0.8, 0.1), createColorMsg(0.6, 0.0, 0.6), v.w());

        marker.colors.push_back(color);
      }
    }
  }

  if (!marker.points.empty())
    markers_.markers.push_back(marker);
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::GroundContactEstimationVis, l3_footstep_planning::PlanningVisPlugin)
