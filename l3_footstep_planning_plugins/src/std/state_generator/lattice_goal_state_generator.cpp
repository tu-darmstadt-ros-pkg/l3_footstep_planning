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
  // get parameters
  getParam("base_idx", base_idx_, BaseInfo::MAIN_BODY_IDX, true);
  getParam("expand_neutral_stance", expand_neutral_stance_, false, true);

  getParam("max_dist", max_dist_, 0.5, true);
  max_dist_sq_ = max_dist_ * max_dist_;
  getParam("min_curve_radius", min_curve_radius_, 0.5, true);

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
    return std::list<StateGenResult>{ result };
  }

  FloatingBase::ConstPtr current_base = current.getState()->getFloatingBase(l3::BaseInfo::MAIN_BODY_IDX);
  FloatingBase::ConstPtr target_base = target.getState()->getFloatingBase(l3::BaseInfo::MAIN_BODY_IDX);

  Transform dstep = FloatingBase::getDelta2D(*current_base, *target_base);

  // do not try to compute arc when 2D euclidian distance is too high
  if (l3::norm_sq(dstep.x(), dstep.y()) > max_dist_sq_)
    return std::list<StateGenResult>{ result };

  // compute curve
  double dyaw = 0.0;
  double radius = 0.0;
  bool is_straight = !computeCircle(dstep.x(), dstep.y(), dyaw, radius);

  /// check curve conditions
  // target can be reached by just driving forward but orientation can mismatch
  if (is_straight)
  {
    result.floating_base = makeShared<FloatingBase>(*target_base);
    result.floating_base->setYaw(current_base->yaw());
  }
  // driving a curve is possible but orientation can mismatch
  else if (radius >= min_curve_radius_ && dyaw * radius <= max_dist_)
  {
    result.floating_base = makeShared<FloatingBase>(*target_base);
    result.floating_base->setYaw(current_base->yaw() + dyaw);
  }

  /// expand neutral stance
  if (result.floating_base && expand_neutral_stance_)
  {
    // generate neutral stance based on discretize floating base for more consistent results
    result.footholds = getNeutralStance(*result.floating_base);

    // copy old heights (to be updated by terrain model later)
    for (Foothold::Ptr f : result.footholds)
    {
      Foothold::ConstPtr f_next = target.getState()->getFoothold(f->idx);
      if (f_next)
        f->setZ(f_next->z());
    }
  }

  return std::list<StateGenResult>{ result };
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::LatticeGoalStateGenerator, l3_footstep_planning::StateGeneratorPlugin)
