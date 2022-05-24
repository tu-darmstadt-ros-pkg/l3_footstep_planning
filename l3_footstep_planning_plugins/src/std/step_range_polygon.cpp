#include <l3_footstep_planning_plugins/std/step_range_polygon.h>

#include <l3_libs/yaml_parser.h>

namespace l3_footstep_planning
{
StepRangePolygon::StepRangePolygon() {}

StepRangePolygon::~StepRangePolygon() {}

bool StepRangePolygon::fromYaml(XmlRpc::XmlRpcValue& p, const DiscreteResolution& res)
{
  if (!parseEdges(p, res))
    return false;

  if (!generateRechabilityMap())
    return false;

  return true;
}

bool StepRangePolygon::pointWithinPolygon(int x, int y) const
{
  x -= min_x;
  y -= min_y;

  if (step_range_.has(x, y))
    return step_range_.at(x, y);
  else
    return false;
}

std::string StepRangePolygon::toString() const
{
  std::string msg;

  // Header line
  msg += "     ";
  for (int x = min_x; x <= max_x; x++)
  {
    if (x == -1)
      msg += "- ";
    else if (x == 0)
      msg += "0 ";
    else if (x == 1)
      msg += "+ ";
    else
      msg += "  ";
  }
  msg += "\n";

  for (int y = max_y; y >= min_y; y--)
  {
    if (y >= 0)
      msg += " ";
    if (std::abs(y) <= 9)
      msg += " ";
    msg += boost::lexical_cast<std::string>(y) + ": ";

    for (int x = min_x; x <= max_x; x++)
      msg += pointWithinPolygon(x, y) ? "+ " : "- ";

    msg += "\n";
  }

  return msg;
}

bool StepRangePolygon::parseAngles(XmlRpc::XmlRpcValue& p, const std::string& key, const DiscreteResolution& res, int& min_angle, int& max_angle)
{
  std::vector<double> angle_limits;
  if (p.hasMember(key) && l3::getYamlValue(p, key, angle_limits))
  {
    if (angle_limits.size() != 2)
    {
      ROS_ERROR("Angle limits must be given as array of two values (min, max), for key %s", key.c_str());
      return false;
    }

    bool mirror_angle = false;
    std::string mirror_key = "mirror_" + key;
    if (p.hasMember(mirror_key))
      l3::getYamlValue(p, mirror_key, mirror_angle);

    if (mirror_angle)
    {
      min_angle = res.toDiscAngle(-angle_limits[1]);
      max_angle = res.toDiscAngle(-angle_limits[0]);
    }
    else
    {
      min_angle = res.toDiscAngle(angle_limits[0]);
      max_angle = res.toDiscAngle(angle_limits[1]);
    }
  }
  else
  {
    min_angle = -std::numeric_limits<int>::max();
    max_angle = std::numeric_limits<int>::max();
  }

  return true;
}

bool StepRangePolygon::parseEdges(XmlRpc::XmlRpcValue& p, const DiscreteResolution& res)
{
  // get params
  bool mirror_x = false;
  if (p.hasMember("mirror_x"))
    l3::getYamlValue(p, "mirror_x", mirror_x);

  bool mirror_y = false;
  if (p.hasMember("mirror_y"))
    l3::getYamlValue(p, "mirror_y", mirror_y);

  // get step range array
  std::vector<double> step_range_x;
  std::vector<double> step_range_y;

  if (!l3::getYamlValue(p, "x", step_range_x) || !l3::getYamlValue(p, "y", step_range_y))
    return false;

  if (step_range_x.size() != step_range_y.size())
  {
    ROS_ERROR("Array size for x and y must be equal!");
    return false;
  }

  // parse edges
  points.clear();
  points.reserve(step_range_x.size());

  double min_x = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();

  double min_y = std::numeric_limits<double>::max();
  double max_y = -std::numeric_limits<double>::max();

  for (size_t i = 0; i < step_range_x.size(); i++)
  {
    double x = static_cast<double>(step_range_x[i]);
    if (mirror_x)
      x = -x;

    double y = static_cast<double>(step_range_y[i]);
    if (mirror_y)
      y = -y;

    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);

    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);

    points.push_back(Point(res.toDiscX(x), res.toDiscY(y)));
  }

  // save statistics
  this->min_x = res.toDiscX(min_x);
  this->max_x = res.toDiscX(max_x);

  this->min_y = res.toDiscY(min_y);
  this->max_y = res.toDiscY(max_y);

  max_x = std::max(std::abs(max_x), std::abs(min_x));
  max_y = std::max(std::abs(max_y), std::abs(min_y));
  max_step_range_width_sq = max_x * max_x + max_y * max_y;
  max_step_range_width = sqrt(max_step_range_width_sq);

  // extract roll limits
  if (!parseAngles(p, "roll", res, this->min_roll, this->max_roll))
    return false;

  // extract pitch limits
  if (!parseAngles(p, "pitch", res, this->min_pitch, this->max_pitch))
    return false;

  // extract yaw limits
  if (!parseAngles(p, "yaw", res, this->min_yaw, this->max_yaw))
    return false;

  return true;
}

bool StepRangePolygon::generateRechabilityMap()
{
  size_t step_range_size_x = max_x - min_x + 1;
  size_t step_range_size_y = max_y - min_y + 1;

  step_range_.init(step_range_size_x, step_range_size_y);

  // compute reachability polygon
  for (int y = min_y; y <= max_y; y++)
  {
    for (int x = min_x; x <= max_x; x++)
    {
      step_range_.at(x - min_x, y - min_y) = l3::isPointWithinPolygon(x, y, points);
    }
  }

  return true;
}
}  // namespace l3_footstep_planning
