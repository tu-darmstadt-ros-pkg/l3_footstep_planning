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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_GROUND_CONTACT_POST_PROCESS_H__
#define L3_FOOTSTEP_PLANNING_PLUGINS_GROUND_CONTACT_POST_PROCESS_H__

#include <ros/ros.h>

#include <l3_footstep_planning_plugins/base/post_process_plugin.h>

namespace l3_footstep_planning
{
class GroundContactPostProcess : public PostProcessPlugin
{
public:
  GroundContactPostProcess();

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  bool postProcess(Foothold& foothold) const override;

protected:
  bool getFootContactSupport(const Foothold& foothold, double& support, Vector4Array& checked_positions) const;

  bool getFootContactSupport(const Foothold& foothold, double& support, unsigned int num_sampling_steps_x, unsigned int num_sampling_steps_y, Vector4Array& checked_positions) const;

  std::map<FootIndex, Vector3> foot_size_map_;

  // ground contact support estimation parameters
  unsigned int min_sampling_steps_x_;    // min number of sampling steps in y
  unsigned int min_sampling_steps_y_;    // min number of sampling steps in y
  unsigned int max_sampling_steps_x_;    // max number of sampling steps in y
  unsigned int max_sampling_steps_y_;    // max number of sampling steps in y
  double max_intrusion_z_;               // how deep the foot may intrude into other objects (z axis only)#
  double max_ground_clearance_;          // maximal distance before a point is treated as "in the air"
  double intrusion_norm_factor_;         // factor to normalize height diff into -1...0 range
  double ground_clearance_norm_factor_;  // factor to normalize height diff into  0...1 range
  double min_contact_support_;           // minimal percentage of foot sole area which must touch ground
};
}  // namespace l3_footstep_planning

#endif
