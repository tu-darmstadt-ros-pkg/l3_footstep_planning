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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_TERRAIN_MODEL_H__
#define L3_FOOTSTEP_PLANNING_PLUGINS_TERRAIN_MODEL_H__

#include <ros/ros.h>

#include <l3_footstep_planning_plugins/base/terrain_model_plugin.h>

#include <l3_terrain_model/terrain_model.h>

namespace l3_footstep_planning
{
class BasicTerrainModel : public TerrainModelPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<BasicTerrainModel> Ptr;
  typedef l3::SharedPtr<BasicTerrainModel> ConstPtr;

  BasicTerrainModel(const std::string& name = "basic_terrain_model");

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

  void reset() override;

  bool isTerrainModelAvailable() const override;

  inline double getResolution() const override { return terrain_model_->getResolution(); }

  inline TerrainResult getHeight(double x, double y, double& height) const override { return terrain_model_->getHeight(x, y, height) ? OK : NO_HEIGHT; }

  inline TerrainResult getNormal(const Pose& pose, Vector3& normal) const override { return terrain_model_->getNormal(pose, normal) ? OK : NO_NORMAL; }
  inline TerrainResult getNormal(const Foothold& foothold, Vector3& normal) const override { return terrain_model_->getNormal(foothold, normal) ? OK : NO_NORMAL; }

protected:
  void setTerrainModel(l3_terrain_modeling::TerrainModelMsg::ConstPtr terrain_model);
  void setGridMap(grid_map_msgs::GridMap::ConstPtr grid_map);

  virtual void notifyTerrainModelUpdate() {}

  l3_terrain_modeling::TerrainModel::Ptr terrain_model_;

  // subscribers
  ros::Subscriber terrain_model_sub_;
  ros::Subscriber grid_map_sub_;

  // publisher
  ros::Publisher terrain_model_pub_;
};
}  // namespace l3_footstep_planning

#endif
