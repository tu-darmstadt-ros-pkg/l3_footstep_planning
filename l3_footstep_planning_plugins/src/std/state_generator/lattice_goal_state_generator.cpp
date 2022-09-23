#include <l3_footstep_planning_plugins/std/state_generator/lattice_goal_state_generator.h>

#include <l3_libs/robot_description/base_info.h>

#include <l3_footstep_planning_libs/modeling/state_space_manager.h>

#include <l3_plugins/robot_model.h>

namespace l3_footstep_planning
{
LatticeGoalStateGenerator::LatticeGoalStateGenerator()
  : StateGeneratorPlugin("lattice_goal_state_generator")
{}

bool LatticeGoalStateGenerator::loadParams(const ParameterSet& params)
{
  if (!StateGeneratorPlugin::loadParams(params))
    return false;

  // get parameters
  base_idx_ = param("base_idx", BaseInfo::MAIN_BODY_IDX, true);
  expand_neutral_stance_ = param("expand_neutral_stance", false, true);

  turn_in_place_ = param("turn_in_place", false, true);
  omni_directional_ = param("omni_directional", false, true);
  move_backwards_ = param("move_backwards", true, true);

  const vigir_generic_params::ParameterSet& p = getSubset("max_dist");
  max_dist_.x() = p.param("x", 0.5);
  max_dist_.y() = p.param("y", 0.5);

  max_dyaw_ = param("max_dyaw", M_PI_4, true);
  min_curve_radius_ = param("min_curve_radius", 0.5, true);

  // read resolution
  planner_res_ = DiscreteResolution(params.getSubset("resolution"));

  return true;
}

std::list<StateGenResult> LatticeGoalStateGenerator::generateNearStateResults(const PlanningState& current, const PlanningState& target,
                                                                              const ExpandStatesIdx& state_expansion_idx) const
{
  StateGenResult result;

  // check for existence of a floating base
  if (!target.getState()->hasFloatingBases())
  {
    ROS_ERROR_NAMED(getName(), "[%s] Goal state must contain a floating base!", getName().c_str());
    return std::list<StateGenResult>();
  }

  // check possible splines between current and target state
  FloatingBase::ConstPtr current_base = current.getState()->getFloatingBase(base_idx_);
  FloatingBase::ConstPtr target_base = target.getState()->getFloatingBase(base_idx_);

  Transform dstep = FloatingBase::getDelta2D(*current_base, *target_base);

  // do not try to compute arc when point is not in ellipse
  if (!l3::isPointInEllipse(l3::Point(dstep.x(), dstep.y(), 0.0), l3::Point(), max_dist_, 1.0, 0.0))
    return std::list<StateGenResult>();

  /// check possible splines

  // reject any backwards movements
  if (!move_backwards_ && dstep.x() < 0.0)
    return std::list<StateGenResult>();

  // only turn in place would be required
  if (std::abs(dstep.x()) < 0.01 && std::abs(dstep.y()) < 0.01)
  {
    if (turn_in_place_)
      result.floating_base = makeShared<FloatingBase>(*target_base);
  }
  else
  {
    // compute curve
    double dyaw = 0.0;
    double radius = 0.0;
    bool is_straight = !computeCircle(dstep.x(), dstep.y(), dyaw, radius);

    // reject too large turning
    if (std::abs(dstep.yaw()) > max_dyaw_)
      return std::list<StateGenResult>();

    // target can be reached by just driving forward
    if (is_straight)
    {
      result.floating_base = makeShared<FloatingBase>(*target_base);

      // if cannot move omnidirectional, then final orientation will mismatch
      if (!omni_directional_)
      {
        // do not propose dead end states
        if (!turn_in_place_ && std::abs(dstep.yaw()) > 0.01)
          return std::list<StateGenResult>();
        else
          result.floating_base->setYaw(current_base->yaw());
      }
    }
    // driving a curve is possible
    else if (radius >= min_curve_radius_)
    {
      result.floating_base = makeShared<FloatingBase>(*target_base);

      // if cannot move omnidirectional, then final orientation will mismatch
      if (!omni_directional_)
      {
        // do not propose dead end states
        if (!turn_in_place_ && std::abs(dstep.yaw() - dyaw) > 0.01)
          return std::list<StateGenResult>();
        else
          result.floating_base->setYaw(current_base->yaw() + dyaw);
      }
    }
  }

  /// expand neutral stance
  if (result.floating_base && expand_neutral_stance_)
    result.footholds = expandNeutralStance(result.floating_base, target.getState(), planner_res_);

  return std::list<StateGenResult>{ result };
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::LatticeGoalStateGenerator, l3_footstep_planning::StateGeneratorPlugin)
