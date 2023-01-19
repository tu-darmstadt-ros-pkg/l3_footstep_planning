//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef ENVIRONMENT_PARAMETERS_H__
#define ENVIRONMENT_PARAMETERS_H__

#include <ros/ros.h>

#include <vigir_generic_params/parameter_manager.h>

#include <l3_footstep_planning_libs/typedefs.h>

namespace l3_footstep_planning
{
struct EnvironmentParameters
{
  // typedefs
  typedef l3::SharedPtr<EnvironmentParameters> Ptr;
  typedef l3::SharedPtr<EnvironmentParameters> ConstPtr;

  EnvironmentParameters(const vigir_generic_params::ParameterSet& params);
  virtual ~EnvironmentParameters();

  double max_risk;

  DiscreteResolution res;  // Discretization resolution

  bool forward_search;   // Whether to use forward search (1) or backward search (0).
  unsigned int num_random_nodes;  // number of random neighbors for R*
  double random_node_distance;

  double heuristic_scale;  // Scaling factor of heuristic, in case it underestimates by a constant factor.

  bool search_until_first_solution;

  /// default max planning time if not given by request
  double max_planning_time;
  double initial_eps;
  double decrease_eps;

  std::string planner_type;

  double feedback_rate;
  int threads;
  unsigned int jobs_per_thread;
};
}  // namespace l3_footstep_planning

#endif
