#include <l3_footstep_planning_plugins/std/state_generator/polygonal_state_generator.h>

#include <l3_libs/yaml_parser.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_plugins/std/step_range_polygon.h>

namespace l3_footstep_planning
{
PolygonalStateGenerator::PolygonalStateGenerator(const std::string& name)
  : StateGeneratorPlugin(name)
{}

bool PolygonalStateGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!StateGeneratorPlugin::loadParams(params))
    return false;

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
    ROS_ERROR_NAMED("PolygonalStateGenerator", "[PolygonalStateGenerator] Parameter 'reachability_polygons/feet' must be given as array (currently '%s')!",
                    l3::toString(p.getType()).c_str());
    return false;
  }

  foot_step_sets_.clear();

  // iterate all feet
  for (size_t i = 0; i < p.size(); i++)
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
          foot_step_sets_[foot_idx].push_back(FootStep(neutral_stance, foot_idx, res.toContX(x), res.toContY(y), res.toContAngle(yaw), 0.0, res));
      }
    }

    //    // output for matlab
    //    if (true)
    //    {
    //      std::list<std::pair<int, int>> points1i;
    //      std::list<std::pair<double, double>> points2d;
    //      std::list<std::pair<double, double>> points3d;

    //      double yaw = 0.0;

    //      State state;
    //      for (Foothold f : RobotModel::getNeutralStance())
    //        state.updateFoothold(FootholdHashed::make(f));

    //      for (const Footstep& f : footstep_sets_[foot_idx])
    //      {
    //        Foothold::Ptr pred = f.getPredFoothold(state);

    //        // Transform delta = Foothold::getDelta(*foothold, *succ);

    //        //points1i.push_back(std::pair<int, int>(res.toDiscX(succ->x()), res.toDiscY(succ->y())));
    //        points3d.push_back(std::pair<double, double>(pred->x(), pred->y()));

    //        Foothold::Ptr succ = f.getSuccFoothold(state);
    //        points2d.push_back(std::pair<double, double>(succ->x(), succ->y()));
    //      }

    //      std::list<std::pair<int, int>> points3i;
    //      std::list<std::pair<double, double>> points4d;
    //      points3i.clear();
    //      for (int y = polygon.min_y; y <= polygon.max_y; y++)
    //      {
    //        for (int x = polygon.min_x; x <= polygon.max_x; x++)
    //        {
    //          if (!polygon.pointWithinPolygon(x, y))
    //            continue;

    //          double global_theta = res.toContYaw(res.toDiscYaw(yaw));

    //          double theta_cos = cos(global_theta);
    //          double theta_sin = sin(global_theta);

    //          double x_d = res.toContX(x);
    //          double y_d = res.toContY(y);

    //          double footstep_x = theta_cos * x_d - theta_sin * y_d;
    //          double footstep_y = theta_sin * x_d + theta_cos * y_d;

    //          int x_i = static_cast<int>(floor(footstep_x / res.toCont().x) + 0.5);
    //          int y_i = static_cast<int>(floor(footstep_y / res.toCont().x) + 0.5);
    //          // int x_i = static_cast<int>(round(footstep_x / cell_size));
    //          // int y_i = static_cast<int>(round(footstep_y / cell_size));

    //          points3i.push_back(std::pair<int, int>(x_i, y_i));
    //          points4d.push_back(std::pair<double, double>(footstep_x, footstep_y));
    //        }
    //      }

    //      ROS_WARN("%s", RobotModel::description()->getFootInfo(foot_idx).name.c_str());

    //      std::string line1;
    //      std::string line2;
    //      for (auto& p : points1i)
    //      {
    //        line1 += boost::lexical_cast<std::string>(p.first) + " ";
    //        line2 += boost::lexical_cast<std::string>(p.second) + " ";
    //      }
    //      //ROS_WARN_STREAM(std::endl << "P1 = [" << line1 << "; " << line2 << "];");

    //      line1.clear();
    //      line2.clear();
    //      for (auto& p : points2d)
    //      {
    //        line1 += boost::str(boost::format("%f") % p.first) + " ";
    //        line2 += boost::str(boost::format("%f") % p.second) + " ";
    //      }
    //      ROS_WARN_STREAM(std::endl << "SUCC = [" << line1 << "; " << line2 << "];");

    //      line1.clear();
    //      line2.clear();
    //      for (auto& p : points3d)
    //      {
    //        line1 += boost::str(boost::format("%f") % p.first) + " ";
    //        line2 += boost::str(boost::format("%f") % p.second) + " ";
    //      }
    //      ROS_WARN_STREAM(std::endl << "PRED = [" << line1 << "; " << line2 << "];");

    //      line1.clear();
    //      line2.clear();
    //      for (auto& p : points3i)
    //      {
    //        line1 += boost::lexical_cast<std::string>(p.first) + " ";
    //        line2 += boost::lexical_cast<std::string>(p.second) + " ";
    //      }
    //      //ROS_WARN_STREAM(std::endl << "P3 = [" << line1 << "; " << line2 << "];");

    //      line1.clear();
    //      line2.clear();
    //      for (auto& p : points4d)
    //      {
    //        line1 += boost::str(boost::format("%f") % p.first) + " ";
    //        line2 += boost::str(boost::format("%f") % p.second) + " ";
    //      }
    //      //ROS_WARN_STREAM(std::endl << "P4 = [" << line1 << "; " << line2 << "];");
    //    }
  }

  return true;
}

std::list<StateGenResult> PolygonalStateGenerator::generatePredStateResults(const PlanningState& state, const State& start, const State& goal,
                                                                            const ExpandStatesIdx& state_expansion_idx) const
{
  return generatePredFootholds(state, state_expansion_idx, foot_step_sets_);
}

std::list<StateGenResult> PolygonalStateGenerator::generateSuccStateResults(const PlanningState& state, const State& start, const State& goal,
                                                                            const ExpandStatesIdx& state_expansion_idx) const
{
  return generateSuccFootholds(state, state_expansion_idx, foot_step_sets_);
}

std::list<StateGenResult> PolygonalStateGenerator::generatePredFootholds(const PlanningState& state, const ExpandStatesIdx& state_expansion_idx,
                                                                         const FootStepSetMap& footsteps) const
{
  std::list<StateGenResult> result;

  // generate new footholds for each foot
  for (const FootIndex& idx : state_expansion_idx.foot_idx)
  {
    // do only consider specific foot ids when given
    if (ignoreFootIdx(idx))
      continue;

    // get pred footsteps
    FootStepSetMap::const_iterator itr = footsteps.find(idx);
    if (itr == footsteps.end())
      continue;

    // generate all states within reachability polygon
    FootholdPtrArray footholds;
    for (const FootStep& footstep : itr->second)
      footholds.push_back(footstep.getPredecessor(state.getState()));

    // trivial case: only one leg is moved -> return list of list each with a single foothold
    if (result.empty())
    {
      for (Foothold::Ptr foothold : footholds)
        result.push_back(StateGenResult(FootholdPtrArray{ foothold }));
    }
    // otherwise permute result
    else
    {
      std::list<StateGenResult> tmp = result;
      result.clear();

      for (const StateGenResult& state_gen_result : tmp)
      {
        /// @TODO: Find more efficient solution
        for (Foothold::Ptr foothold : footholds)
        {
          FootholdPtrArray a = state_gen_result.footholds;
          a.push_back(foothold);
          result.push_back(StateGenResult(a));
        }
      }
    }
  }

  return result;
}

std::list<StateGenResult> PolygonalStateGenerator::generateSuccFootholds(const PlanningState& state, const ExpandStatesIdx& state_expansion_idx,
                                                                         const FootStepSetMap& footsteps) const
{
  std::list<StateGenResult> result;

  // generate new footholds for each foot
  for (const FootIndex& idx : state_expansion_idx.foot_idx)
  {
    // do only consider specific foot ids when given
    if (ignoreFootIdx(idx))
      continue;

    // get succ footsteps
    FootStepSetMap::const_iterator itr = footsteps.find(idx);
    if (itr == footsteps.end())
      continue;

    // generate all states within reachability polygon
    FootholdPtrArray footholds;
    for (const FootStep& footstep : itr->second)
      footholds.push_back(footstep.getSuccessor(state.getState()));

    // trivial case: only one leg is moved -> return list of list each with a single foothold
    if (result.empty())
    {
      for (Foothold::Ptr foothold : footholds)
        result.push_back(StateGenResult(FootholdPtrArray{ foothold }));
    }
    // otherwise permute result
    else
    {
      std::list<StateGenResult> tmp = result;
      result.clear();

      for (const StateGenResult& arr : tmp)
      {
        /// @TODO: Find more efficient solution
        for (Foothold::Ptr foothold : footholds)
        {
          FootholdPtrArray a = arr.footholds;
          a.push_back(foothold);
          result.push_back(StateGenResult(a));
        }
      }
    }
  }

  return result;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::PolygonalStateGenerator, l3_footstep_planning::StateGeneratorPlugin)
