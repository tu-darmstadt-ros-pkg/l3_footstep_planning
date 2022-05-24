#include <l3_footstep_planning_plugins/std/robot_model/support_polygon_check.h>

#include <l3_libs/yaml_parser.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_plugins/std/step_range_polygon.h>

namespace l3_footstep_planning
{
SupportPolygonCheck::SupportPolygonCheck()
  : ReachabilityPlugin("support_polygon_check")
{}

bool SupportPolygonCheck::loadParams(const ParameterSet& params)
{
  if (!ReachabilityPlugin::loadParams(params))
    return false;

  full_check_ = param("full_check", false, true);

  /// get polygons
  // check parameter format
  XmlRpc::XmlRpcValue p = param("support_polygons/feet", XmlRpc::XmlRpcValue());

  if (p.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("[%s] Parameter 'support_polygons/feet' must be given as array (currently '%s')!", getName().c_str(), l3::toString(p.getType()).c_str());
    return false;
  }

  bool print_polygons = param("print_polygons", false, true);

  polygons_.clear();

  // iterate through all feet
  for (size_t i = 0; i < p.size(); i++)
  {
    FootIndex foot_idx;
    if (!getYamlValue(p[i], "idx", foot_idx))
      return false;

    if (!parsePoints(p[i], polygons_[foot_idx]))
      return false;

    if (print_polygons)
    {
      FootInfo foot_info;
      if (!RobotModel::description()->getFootInfo(foot_idx, foot_info))
      {
        ROS_ERROR_NAMED("ReachabilityPolygon", "[ReachabilityPolygon] Unknown foot id '%u' sepcified!", foot_idx);
        return false;
      }

      DiscreteResolution res(params.getSubset("resolution"));

      StepRangePolygon step_range;
      if (!step_range.fromYaml(p[i], res))
        return false;

      ROS_INFO_STREAM("Support Polygon [" << foot_info.name << " (" << foot_idx << ")]:\n" << step_range);
    }
  }

  return true;
}

bool SupportPolygonCheck::isReachable(const PlanningState& state) const
{
  if (full_check_)
    return isReachable(*state.getState());
  else
  {
    for (const Step::StepDataPair& p : state.getStep()->getStepDataMap())
    {
      StepData::ConstPtr step_data = p.second;
      Foothold::ConstPtr foothold = forwardSearch() ? step_data->target : step_data->origin;
      if (!isReachable(*foothold))
        return false;
    }
  }

  return true;
}

bool SupportPolygonCheck::isReachable(const State& state) const
{
  for (Foothold::ConstPtr foothold : state.getFootholds())
  {
    if (!isReachable(*foothold))
      return false;
  }
  return true;
}

bool SupportPolygonCheck::isReachable(const Foothold& foothold) const
{
  std::unordered_map<FootIndex, PointArray>::const_iterator itr = polygons_.find(foothold.idx);
  if (itr == polygons_.end())
    return true;

  const PointArray& polygon = itr->second;

  //  PointArray support;
  //  if (!foothold.data.get("support_polygon", support))
  //    return true;

  //  if (support.size() < 3)
  //    return true;

  PointArray facets;
  if (!foothold.data.get("mcp_facets", facets))
    return true;

  if (facets.size() < 3)
    return true;

  for (const Point& p : polygon)
  {
    /// @todo: requires sorted points
    //    if (!pointWithinPolygon(p.x(), p.y(), support))
    //      return false;

    // find facet that includes the point
    bool found = false;
    for (size_t i = 0; i < facets.size() / 3; i++)
    {
      if (isPointInTriangle(foothold.pose() * p, facets[i*3], facets[i*3+1], facets[i*3+2]))
      {
        found = true;
        break;
      }
    }

    if (!found)
      return false;
  }

  return true;
}

bool SupportPolygonCheck::parsePoints(XmlRpc::XmlRpcValue& p, PointArray& points)
{
  // get params
  bool mirror_x = false;
  if (p.hasMember("mirror_x"))
    getYamlValue(p, "mirror_x", mirror_x);

  bool mirror_y = false;
  if (p.hasMember("mirror_y"))
    getYamlValue(p, "mirror_y", mirror_y);

  // get step range array
  std::vector<double> x_array;
  std::vector<double> y_array;

  if (!getYamlValue(p, "x", x_array) || !getYamlValue(p, "y", y_array))
    return false;

  if (x_array.size() != y_array.size())
  {
    ROS_ERROR("[%s] Array size for x and y must be equal!", getName().c_str());
    return false;
  }

  for (size_t i = 0; i < x_array.size(); i++)
  {
    double x = x_array[i];
    if (mirror_x)
      x = -x;

    double y = y_array[i];
    if (mirror_y)
      y = -y;

    points.push_back(Point(x, y, 0.0));
  }

  return true;
}

}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::SupportPolygonCheck, l3_footstep_planning::ReachabilityPlugin)
