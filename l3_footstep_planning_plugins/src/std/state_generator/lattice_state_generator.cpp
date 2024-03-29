#include <l3_footstep_planning_plugins/std/state_generator/lattice_state_generator.h>

#include <l3_libs/yaml_parser.h>

#include <l3_math/math.h>

#include <l3_plugins/robot_model.h>

namespace l3_footstep_planning
{
LatticeStateGenerator::LatticeStateGenerator(const std::string& name)
  : StateGeneratorPlugin(name)
{}

bool LatticeStateGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!StateGeneratorPlugin::loadParams(params))
    return false;

  // get parameters
  getParam("base_idx", base_idx_, BaseInfo::MAIN_BODY_IDX, true);
  getParam("expand_neutral_stance", expand_neutral_stance_, false, true);

  // read resolution
  planner_res_ = DiscreteResolution(params.getSubset("resolution"));
  DiscreteResolution lattice_res = planner_res_;

  // use different resolution for generating motion primitives if given
  if (hasParam("resolution"))
    lattice_res = DiscreteResolution(getSubset("resolution"));

  // generate motion primitives
  motion_primitives_.clear();
  if (!generateMotionPrimitives(lattice_res))
    return false;

  ROS_INFO("[%s] %lu Motion Primitives", getName().c_str(), motion_primitives_.size());

  return true;
}

std::list<StateGenResult> LatticeStateGenerator::generatePredStateResults(const PlanningState& state, const State& start, const State& goal,
                                                                          const ExpandStatesIdx& state_expansion_idx) const
{
  std::list<StateGenResult> result;

  // check if we should even expand
  bool expand = false;
  for (const BaseIndex& idx : state_expansion_idx.floating_base_idx)
  {
    if (idx == base_idx_)
    {
      expand = true;
      break;
    }
  }

  if (!expand)
    return result;

  // perform state expansion
  for (const FloatingBaseStepAction& step : motion_primitives_)
  {
    StateGenResult gen_state;
    FloatingBase::Ptr floating_base = step.getPredecessor(state.getState());

    // generate neutral stance based on discretize floating base for more consistent results
    if (expand_neutral_stance_)
      gen_state.footholds = expandNeutralStance(floating_base, state.getState(), planner_res_);

    gen_state.floating_base = floating_base;

    result.push_back(gen_state);
  }

  return result;
}

std::list<StateGenResult> LatticeStateGenerator::generateSuccStateResults(const PlanningState& state, const State& start, const State& goal,
                                                                          const ExpandStatesIdx& state_expansion_idx) const
{
  std::list<StateGenResult> result;

  // check if we should even expand
  bool expand = false;
  for (const BaseIndex& idx : state_expansion_idx.floating_base_idx)
  {
    if (idx == base_idx_)
    {
      expand = true;
      break;
    }
  }

  if (!expand)
    return result;

  // perform state expansion
  for (const FloatingBaseStepAction& step : motion_primitives_)
  {
    StateGenResult gen_state;
    FloatingBase::Ptr floating_base = step.getSuccessor(state.getState());

    // generate neutral stance based on discretize floating base for more consistent results
    if (expand_neutral_stance_)
      gen_state.footholds = expandNeutralStance(floating_base, state.getState(), planner_res_);

    gen_state.floating_base = floating_base;

    result.push_back(gen_state);
  }

  return result;
}
bool LatticeStateGenerator::generateMotionPrimitives(const DiscreteResolution& lattice_res)
{
  // parameters used for motion primitive generation
  bool turn_in_place = param("turn_in_place", false, true);
  bool omni_directional = param("omni_directional", false, true);
  bool move_backwards = param("move_backwards", true, true);

  const vigir_generic_params::ParameterSet& p = getSubset("max_dist");
  l3::Point max_dist;
  max_dist.x() = p.param("x", 0.5);
  max_dist.y() = p.param("y", 0.5);

  double max_dyaw = param("max_dyaw", M_PI_4, true);

  double min_curve_radius = param("min_curve_radius", 0.5, true);

  // sample area of interest
  for (int y = lattice_res.toDiscY(-max_dist.y()); y <= lattice_res.toDiscY(max_dist.y()); y++)
  {
    for (int x = move_backwards ? lattice_res.toDiscX(-max_dist.x()) : 0; x <= lattice_res.toDiscX(max_dist.x()); x++)
    {
      // check if within ellipse
      if (!l3::isPointInEllipse(l3::Point(lattice_res.toContX(x), lattice_res.toContY(y), 0.0), l3::Point(), max_dist, 1.0, 0.0))
        continue;

      double dx = lattice_res.toContX(x);
      double dy = lattice_res.toContY(y);
      double dyaw = 0.0;
      double radius = 0.0;

      // add omnidirectional primitive
      if (omni_directional && dy != 0.0)
        motion_primitives_.push_back(FloatingBaseStepAction(Pose(), base_idx_, dx, dy, 0.0, 0.0, 0.0, lattice_res));

      // check if radius is too sharp
      bool is_circle = computeCircle(dx, dy, dyaw, radius);
      if (is_circle && (radius <= min_curve_radius || dyaw > max_dyaw))
        continue;

      // add straight or curve primitive
      motion_primitives_.push_back(FloatingBaseStepAction(Pose(), base_idx_, dx, dy, 0.0, dyaw, 0.0, lattice_res));
    }
  }

  // generate turn in place motion primitives
  if (turn_in_place)
  {
    for (int yaw = lattice_res.toDiscAngle(-max_dyaw); yaw <= lattice_res.toDiscAngle(max_dyaw); yaw++)
      motion_primitives_.push_back(FloatingBaseStepAction(Pose(), base_idx_, 0.0, 0.0, 0.0, lattice_res.toContAngle(yaw), 0.0, lattice_res));
  }

  return true;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::LatticeStateGenerator, l3_footstep_planning::StateGeneratorPlugin)
