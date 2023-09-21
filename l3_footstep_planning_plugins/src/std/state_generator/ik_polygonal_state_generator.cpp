#include <l3_footstep_planning_plugins/std/state_generator/ik_polygonal_state_generator.h>

#include <l3_libs/yaml_parser.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_plugins/std/step_range_polygon.h>

namespace l3_footstep_planning
{
IKPolygonalStateGenerator::IKPolygonalStateGenerator(const std::string& name)
  : PolygonalStateGenerator(name)
{}

bool IKPolygonalStateGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  //  if (!PolygonalStateGenerator::loadParams(params)) // we do not rely on classic polygonal state generator
  if (!StateGeneratorPlugin::loadParams(params))
    return false;

  if (!RobotModel::kinematics())
  {
    ROS_ERROR("[%s] KinematicsPlugin required!", getName().c_str());
    return false;
  }

  DiscreteResolution res;

  // read resolution
  if (hasParam("resolution"))
    res = DiscreteResolution(getSubset("resolution"));
  // use planner resolution
  else
    res = DiscreteResolution(params.getSubset("resolution"));

  // check parameter format
  XmlRpc::XmlRpcValue p;
  getParam("reachability_polygons/feet", p, XmlRpc::XmlRpcValue());

  if (p.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_NAMED("ReachabilityStateGenerator", "[ReachabilityStateGenerator] Parameter 'reachability_polygons/feet' must be given as array (currently '%s')!",
                    l3::toString(p.getType()).c_str());
    return false;
  }

  // iterate all feet
  for (int i = 0; i < p.size(); i++)
  {
    FootIndex foot_idx;
    if (!l3::getYamlValue(p[i], "idx", foot_idx))
      return false;

    const Pose& neutral_stance = RobotModel::description()->getFootInfo(foot_idx).neutral_stance;

    StepRangePolygon polygon;
    if (!polygon.fromYaml(p[i], res))
      return false;

    for (int y = polygon.min_y; y <= polygon.max_y; y++)
    {
      for (int x = polygon.min_x; x <= polygon.max_x; x++)
      {
        if (!polygon.pointWithinPolygon(x, y))
          continue;

        // sample area
        for (int yaw = polygon.min_yaw; yaw <= polygon.max_yaw; yaw++)
        {
          std::vector<double> q;
          if (!RobotModel::kinematics()->calcLegIK(Foothold(foot_idx, res.toContX(x), res.toContY(y), 0.0, res.toContAngle(yaw)), q))
            continue;

          /// @todo Check Jacobian and define limits next to singularities

          foot_step_actions_[foot_idx].push_back(FootStepAction(neutral_stance, foot_idx, res.toContX(x), res.toContY(y), res.toContAngle(yaw), 0.0, res));
        }
      }
    }
  }

  return true;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::IKPolygonalStateGenerator, l3_footstep_planning::StateGeneratorPlugin)
