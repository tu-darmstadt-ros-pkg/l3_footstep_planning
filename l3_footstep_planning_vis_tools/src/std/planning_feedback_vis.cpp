#include <l3_footstep_planning_vis_tools/std/planning_feedback_vis.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_vis_tools/visualization.h>

namespace l3_footstep_planning
{
PlanningFeedbackVis::PlanningFeedbackVis()
  : PlanningMarkersVisPlugin("planning_feedback_vis")
{}

void PlanningFeedbackVis::clear()
{
  visited_steps_.clear();
  visited_bases_.clear();

  markers_.markers.clear();
  markers_.markers.push_back(createResetMarker("recently_visited_steps"));
  markers_.markers.push_back(createResetMarker("total_visited_steps"));
  markers_.markers.push_back(createResetMarker("recently_visited_bases"));
  markers_.markers.push_back(createResetMarker("total_visited_bases"));
  markers_.markers.push_back(createResetMarker("last_visited_step"));
  markers_.markers.push_back(createResetMarker("current_solution"));
}

void PlanningFeedbackVis::visualize(const msgs::PlanningFeedback& planning_feedback)
{
  // merge to total visited steps
  for (msgs::StepDataArray::const_iterator itr = planning_feedback.visited_steps.begin(); itr != planning_feedback.visited_steps.end(); itr++)
  {
    const msgs::StepData& step = *itr;
    visited_steps_.erase(step);
    visited_steps_.insert(step);
  }

  for (msgs::BaseStepDataArray::const_iterator itr = planning_feedback.visited_bases.begin(); itr != planning_feedback.visited_bases.end(); itr++)
  {
    const msgs::BaseStepData& base_step = *itr;
    visited_bases_.erase(base_step);
    visited_bases_.insert(base_step);
  }

  // handle recently visited steps
  visualizeRecentlyVisitedSteps(planning_feedback);

  // handle total visited steps
  visualizeTotalVisitedSteps(planning_feedback);

  // handle recently visited bases
  visualizeRecentlyVisitedBases(planning_feedback);

  // handle total visited bases
  visualizeTotalVisitedBases(planning_feedback);

  // handle last visited step
  l3::appendMarkerArray(markers_, feetToFootMarkerArray(applyFootIdxWhitelist(planning_feedback.last_visited_state), *RobotModel::description(), "last_visited_step"));

  // handle current step plan
  msgs::StepArray steps;
  for (const msgs::Step& s : planning_feedback.current_step_plan.steps)
    steps.push_back(s);
  l3::appendMarkerArray(markers_, stepPlanToFootMarkerArray(steps, *RobotModel::description(), false, "current_solution"));
}

void PlanningFeedbackVis::visualizeRecentlyVisitedSteps(const msgs::PlanningFeedback& planning_feedback)
{
  if (!planning_feedback.visited_steps.empty())
  {
    visualization_msgs::Marker marker;
    marker.header = planning_feedback.header;
    marker.ns = "recently_visited_steps";
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.0;
    marker.color = l3::createColorMsg(1.0, 1.0, 1.0);

    for (size_t i = 0; i < planning_feedback.visited_steps.size(); i++)
    {
      if (planning_feedback.forward_search)
        marker.points.push_back(planning_feedback.visited_steps[i].target.pose.position);
      else
        marker.points.push_back(planning_feedback.visited_steps[i].origin.pose.position);
    }

    markers_.markers.push_back(marker);
  }
}

void PlanningFeedbackVis::visualizeTotalVisitedSteps(const msgs::PlanningFeedback& planning_feedback)
{
  if (!visited_steps_.empty())
  {
    ros::Time current_time = ros::Time::now();

    visualization_msgs::Marker marker;
    marker.header = planning_feedback.header;
    marker.ns = "total_visited_steps";
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.0;

    for (const msgs::StepData& step_data : visited_steps_)
    {
      if (planning_feedback.forward_search)
      {
        marker.points.push_back(step_data.target.pose.position);
        double age_scale = std::min(5.0, (current_time - step_data.target.header.stamp).toSec()) / 5.0;
        marker.colors.push_back(l3::getColorScale(l3::createColorMsg(1.0, 0.0, 0.0), l3::createColorMsg(0.0, 0.0, 1.0), age_scale));
      }
      else
      {
        marker.points.push_back(step_data.origin.pose.position);
        double age_scale = std::min(5.0, (current_time - step_data.origin.header.stamp).toSec()) / 5.0;
        marker.colors.push_back(l3::getColorScale(l3::createColorMsg(1.0, 0.0, 0.0), l3::createColorMsg(0.0, 0.0, 1.0), age_scale));
      }
    }

    markers_.markers.push_back(marker);
  }
}

void PlanningFeedbackVis::visualizeRecentlyVisitedBases(const msgs::PlanningFeedback& planning_feedback)
{
  if (!planning_feedback.visited_bases.empty())
  {
    visualization_msgs::Marker marker;
    marker.header = planning_feedback.header;
    marker.ns = "recently_visited_bases";
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.0;
    marker.color = l3::createColorMsg(1.0, 1.0, 1.0);

    for (size_t i = 0; i < planning_feedback.visited_bases.size(); i++)
    {
      if (planning_feedback.forward_search)
        marker.points.push_back(planning_feedback.visited_bases[i].target.pose.position);
      else
        marker.points.push_back(planning_feedback.visited_bases[i].origin.pose.position);
    }

    markers_.markers.push_back(marker);
  }
}

void PlanningFeedbackVis::visualizeTotalVisitedBases(const msgs::PlanningFeedback& planning_feedback)
{
  if (!visited_bases_.empty())
  {
    ros::Time current_time = ros::Time::now();

    visualization_msgs::Marker marker;
    marker.header = planning_feedback.header;
    marker.ns = "total_visited_bases";
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.0;

    for (const msgs::BaseStepData& base_step_data : visited_bases_)
    {
      if (planning_feedback.forward_search)
      {
        marker.points.push_back(base_step_data.target.pose.position);
        double age_scale = std::min(5.0, (current_time - base_step_data.target.header.stamp).toSec()) / 5.0;
        marker.colors.push_back(l3::getColorScale(l3::createColorMsg(1.0, 0.0, 0.0), l3::createColorMsg(0.0, 0.0, 1.0), age_scale));
      }
      else
      {
        marker.points.push_back(base_step_data.origin.pose.position);
        double age_scale = std::min(5.0, (current_time - base_step_data.origin.header.stamp).toSec()) / 5.0;
        marker.colors.push_back(l3::getColorScale(l3::createColorMsg(1.0, 0.0, 0.0), l3::createColorMsg(0.0, 0.0, 1.0), age_scale));
      }
    }

    markers_.markers.push_back(marker);
  }
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::PlanningFeedbackVis, l3_footstep_planning::PlanningVisPlugin)
