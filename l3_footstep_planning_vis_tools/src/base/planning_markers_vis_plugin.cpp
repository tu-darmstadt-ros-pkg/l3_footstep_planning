#include <l3_footstep_planning_vis_tools/base/planning_markers_vis_plugin.h>

#include <l3_footstep_planning_vis_tools/visualization.h>

namespace l3_footstep_planning
{
PlanningMarkersVisPlugin::PlanningMarkersVisPlugin(const std::string& name)
  : PlanningVisPlugin(name)
{}

void PlanningMarkersVisPlugin::clear()
{
  markers_.markers.clear();
  markers_.markers.push_back(l3::createResetMarker(marker_ns_));
}
}  // namespace l3_footstep_planning
