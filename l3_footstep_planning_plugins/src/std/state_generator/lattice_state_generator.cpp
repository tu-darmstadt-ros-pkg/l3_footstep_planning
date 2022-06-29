#include <l3_footstep_planning_plugins/std/state_generator/lattice_state_generator.h>

#include <l3_libs/yaml_parser.h>

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

  return true;
}

std::list<StateGenResult> LatticeStateGenerator::generatePredStateResults(const PlanningState& state, const State& start, const State& goal,
                                                                          const ExpandStatesIdx& state_expansion_idx) const
{
  return std::list<StateGenResult>();
}

std::list<StateGenResult> LatticeStateGenerator::generateSuccStateResults(const PlanningState& state, const State& start, const State& goal,
                                                                          const ExpandStatesIdx& state_expansion_idx) const
{
  return std::list<StateGenResult>();
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::LatticeStateGenerator, l3_footstep_planning::StateGeneratorPlugin)
