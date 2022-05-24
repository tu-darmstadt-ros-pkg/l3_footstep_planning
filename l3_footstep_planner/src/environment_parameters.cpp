#include <l3_footstep_planner/environment_parameters.h>

namespace l3_footstep_planning
{
EnvironmentParameters::EnvironmentParameters(const vigir_generic_params::ParameterSet& params)
{
  // get robot specific parameters not included in the parameter set
  // ros::NodeHandle nh;

  // get resolution
  res = DiscreteResolution(params.getSubset("resolution"));

  params.getParam("max_risk", max_risk, 1.0);

  // load remaining parameters from parameter set
  params.getParam("heuristic_scale", heuristic_scale);

  params.getParam("planner_type", planner_type);
  params.getParam("search_until_first_solution", search_until_first_solution);
  params.getParam("forward_search", forward_search);

  params.getParam("max_planning_time", max_planning_time);
  params.getParam("initial_epsilon", initial_eps);
  params.getParam("decrease_epsilon", decrease_eps);

  if (planner_type == "RSTARPlanner")
  {
    params.getParam("num_random_nodes", num_random_nodes);
    //params.getParam("random_node_dist", random_node_distance);
  }

  // misc parameters
  params.getParam("feedback_rate", feedback_rate);
  params.getParam("threads", threads);
  params.getParam("jobs_per_thread", jobs_per_thread);
}

EnvironmentParameters::~EnvironmentParameters() {}
}  // namespace l3_footstep_planning
