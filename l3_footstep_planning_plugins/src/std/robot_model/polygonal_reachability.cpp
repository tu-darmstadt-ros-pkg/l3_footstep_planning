#include <l3_footstep_planning_plugins/std/robot_model/polygonal_reachability.h>

#include <l3_libs/yaml_parser.h>

#include <l3_math/math.h>

#include <l3_plugins/robot_model.h>

namespace l3_footstep_planning
{
PolygonalReachability::PolygonalReachability()
  : ReachabilityPlugin("reachability_polygon")
{}

bool PolygonalReachability::loadParams(const ParameterSet& params)
{
  if (!ReachabilityPlugin::loadParams(params))
    return false;

  // read resolution
  if (hasParam("resolution"))
    res_ = DiscreteResolution(getSubset("resolution"));
  // use planner resolution
  else
    res_ = DiscreteResolution(params.getSubset("resolution"));

  /// get neutral stances
  neutral_stance_.clear();
  for (const FootInfoPair& p : RobotModel::description()->getFootInfoMap())
    neutral_stance_[p.first] = p.second.neutral_stance;

  /// get reachability polygons for feet
  if (!initFootReachability())
    return false;

  /// get reachability polygons for floating bases
  if (!initFloatingBaseReachability())
    return false;

  return true;
}

bool PolygonalReachability::isReachable(const PlanningState& state) const
{
  if (!state.getStep()->hasStepData() && !state.getStep()->hasMovingFloatingBase())
    return isReachable(*state.getState());

  /// Note: We assume that any adjacent state was verified in past and is valid.
  /// Thus, we need only check the newly created state (current) and the approximated
  /// swing state.

  Foothold robot_pose = Foothold(0, state.getState()->getFeetCenter());

  // interpolate robot state during state transition
  Foothold swing_pose;
  const Pose& adj = state.getAdjacentState()->getFeetCenter();
  const Pose& cur = state.getFeetCenter();
  swing_pose.setX(0.5 * (adj.x() + cur.x()));
  swing_pose.setY(0.5 * (adj.y() + cur.y()));
  swing_pose.setYaw(0.5 * (adj.yaw() + cur.yaw()));

  for (const Step::StepDataPair& p : state.getStep()->getStepDataMap())
  {
    StepData::ConstPtr step_data = p.second;

    // check if new (current) state is valid
    Foothold::ConstPtr foothold = forwardSearch() ? step_data->target : step_data->origin;
    Transform delta = Foothold::getDelta2D(robot_pose, *foothold);
    if (!isReachable(foothold->idx, delta.x(), delta.y(), delta.yaw()))
      return false;

    // check if swing state (midstance pose) is valid
    delta = Foothold::getDelta2D(swing_pose, *step_data->target);
    if (!isReachable(step_data->target->idx, delta.x(), delta.y(), delta.yaw()))
      return false;
  }

  for (const Step::BaseStepDataPair& p : state.getStep()->getMovingFloatingBaseMap())
  {
    BaseStepData::ConstPtr base_step_data = p.second;

    FloatingBase::ConstPtr floating_base = forwardSearch() ? base_step_data->target : base_step_data->origin;
    Transform delta = FloatingBase::getDelta2D(*base_step_data->origin, *base_step_data->target);
    if (!isReachable(base_step_data->target->idx, delta.x(), delta.y(), delta.yaw(), 0.0))
      return false;
  }

  return true;
}

bool PolygonalReachability::isReachable(const State& state) const
{
  Foothold robot_pose(0, state.getFeetCenter());

  // checking if all footholds are within the reachability polygon
  for (Foothold::ConstPtr f : state.getFootholds())
  {
    Transform delta = Foothold::getDelta2D(robot_pose, *f);
    if (!isReachable(f->idx, delta.x(), delta.y(), delta.yaw()))
      return false;
  }

  // checking if all floating bases are within their reachability limits
  /// @todo

  return true;
}

bool PolygonalReachability::initFootReachability()
{
  bool print_polygons = param("print_polygons", false, true);

  XmlRpc::XmlRpcValue p;
  getParam("reachability_polygons/feet", p, XmlRpc::XmlRpcValue());

  // check parameter format
  if (p.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_NAMED("ReachabilityPolygon", "[ReachabilityPolygon] Parameter 'reachability_polygons/feet' must be given as array (currently '%s')!",
                    l3::toString(p.getType()).c_str());
    return false;
  }

  foot_polygons_.clear();

  // iterate through all feet
  for (size_t i = 0; i < p.size(); i++)
  {
    FootIndex foot_idx;
    if (!getYamlValue(p[i], "idx", foot_idx))
      return false;

    FootInfo foot_info;
    if (!RobotModel::description()->getFootInfo(foot_idx, foot_info))
    {
      ROS_ERROR_NAMED("ReachabilityPolygon", "[ReachabilityPolygon] Unknown foot id '%u' sepcified!", foot_idx);
      return false;
    }

    StepRangePolygon& step_range = foot_polygons_[foot_idx];
    if (!step_range.fromYaml(p[i], res_))
      return false;

    ROS_INFO_STREAM_COND(print_polygons, "Reachability Polygon [" << foot_info.name << " (" << foot_idx << ")]:\n" << step_range);
  }

  return true;
}

bool PolygonalReachability::initFloatingBaseReachability()
{
  bool print_polygons = param("print_polygons", false, true);

  XmlRpc::XmlRpcValue p;
  if (getParam("reachability_polygons/floating_base", p, XmlRpc::XmlRpcValue(), true))
  {
    // check parameter format
    if (p.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR_NAMED("ReachabilityPolygon", "[ReachabilityPolygon] Parameter 'reachability_polygons/floating_base' must be given as array (currently '%s')!",
                      l3::toString(p.getType()).c_str());
      return false;
    }

    // iterate through all bases
    for (size_t i = 0; i < p.size(); i++)
    {
      BaseIndex base_idx;
      if (!getYamlValue(p[i], "idx", base_idx))
        return false;

      BaseInfo base_info;
      if (!RobotModel::description()->getBaseInfo(base_idx, base_info))
      {
        ROS_ERROR_NAMED("ReachabilityPolygon", "[ReachabilityPolygon] Unknown base id '%u' sepcified!", base_idx);
        return false;
      }

      if (!floating_base_polygons_[base_idx].fromYaml(p[i], res_))
        return false;

      ROS_INFO_STREAM_COND(print_polygons, "Reachability Polygon [" << base_info.name << " (" << base_idx << ")]:\n" << floating_base_polygons_[base_idx]);
    }
  }

  return true;
}

bool PolygonalReachability::isReachable(const FootIndex& foot_idx, double dx, double dy, double dyaw) const
{
  // substract neutral stance
  std::map<FootIndex, Pose>::const_iterator stance_itr = neutral_stance_.find(foot_idx);
  if (stance_itr == neutral_stance_.end())
  {
    ROS_ERROR_THROTTLE(5.0, "[ReachabilityPolygon] No neutral stance for foot index %u available (this message is throttled)!", foot_idx);
    return false;
  }

  const Pose& neutral_stance = stance_itr->second;
  dx -= neutral_stance.x();
  dy -= neutral_stance.y();
  dyaw -= neutral_stance.yaw();

  // get step range polygon
  const FootStepRangeMap::const_iterator range_itr = foot_polygons_.find(foot_idx);
  if (range_itr == foot_polygons_.end())
  {
    ROS_WARN_ONCE("[ReachabilityPolygon] No range polygon given for foot idx %u. Ignoring check request. No further warnings will be messaged.", foot_idx);
    return true;
  }

  const StepRangePolygon& polygon = range_itr->second;

  // check if stance_yaw_diff is not within the executable range
  int disc_dyaw = res_.toDiscAngle(dyaw);
  if (disc_dyaw > polygon.max_yaw || disc_dyaw < polygon.min_yaw)
    return false;

  //  int footstep_x = floor((dx-polygon.min_x) / cell_size_);
  //  int footstep_y = floor((dy-polygon.min_y) / cell_size_);

  return polygon.pointWithinPolygon(res_.toDiscX(dx), res_.toDiscY(dy));
}

bool PolygonalReachability::isReachable(const BaseIndex& base_idx, double dx, double dy, double dyaw, double /*dpitch*/) const
{
  // get step range polygon
  std::map<BaseIndex, StepRangePolygon>::const_iterator itr = floating_base_polygons_.find(base_idx);

  if (itr == floating_base_polygons_.end())
    return true;

  const StepRangePolygon& poly = itr->second;

  // check if stance_yaw_diff is not within the executable range
  int disc_dyaw = res_.toDiscAngle(dyaw);
  if (disc_dyaw > poly.max_yaw || disc_dyaw < poly.min_yaw)
    return false;

  return poly.pointWithinPolygon(res_.toDiscX(dx), res_.toDiscY(dy));
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::PolygonalReachability, l3_footstep_planning::ReachabilityPlugin)
