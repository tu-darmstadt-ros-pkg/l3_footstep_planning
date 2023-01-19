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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_TERRAIN_MODEL_PLUGIN_H__
#define L3_FOOTSTEP_PLANNING_PLUGINS_TERRAIN_MODEL_PLUGIN_H__

#include <ros/ros.h>

#include <vigir_pluginlib/plugin.h>

#include <l3_libs/types/types.h>

#include <l3_math/angles.h>

namespace l3_footstep_planning
{
using namespace l3;

// Range definition for enums:
// error: < 0
// ok: 0
// warnings: > 0
enum TerrainResult
{
  ERROR = -1,
  OK = 0,
  NO_HEIGHT = 1,
  NO_NORMAL = 2,
  NO_DATA = NO_HEIGHT + NO_NORMAL,
};

class TerrainModelPlugin : public virtual vigir_pluginlib::Plugin
{
public:
  // typedefs
  typedef l3::SharedPtr<TerrainModelPlugin> Ptr;
  typedef l3::SharedPtr<const TerrainModelPlugin> ConstPtr;

  TerrainModelPlugin(const std::string& name);

  /**
   * @brief Resets the plugin to initial state.
   */
  virtual void reset() {}

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  bool isUnique() const final { return true; }

  inline SharedLock sharedLockModel() const { return SharedLock(model_lock_); }
  inline UniqueLock uniqueLockModel() const { return UniqueLock(model_lock_); }
  bool isLocked() const;

  virtual bool isTerrainModelAvailable() const = 0;

  /**
   * @brief Returns resolution of underyling model given in [m]
   * @return resolution in [m]
   */
  virtual double getResolution() const = 0;

  virtual TerrainResult getHeight(double x, double y, double& height) const { return NO_HEIGHT; }
  virtual TerrainResult getHeight(const Foothold& foothold, double& height) const { return getHeight(foothold.x(), foothold.y(), height); }

  virtual TerrainResult getNormal(const Pose& pose, Vector3& normal) const { return NO_NORMAL; }
  virtual TerrainResult getNormal(const Foothold& foothold, Vector3& normal) const { return getNormal(foothold.pose(), normal); }
  virtual TerrainResult getNormal(const FloatingBase& floating_base, Vector3& normal) const { return getNormal(floating_base.pose(), normal); }

  virtual TerrainResult update3DData(Pose& pose) const { return update3DDataImpl(pose); }
  virtual TerrainResult update3DData(Foothold& foothold) const;

  template <typename T>
  TerrainResult update3DDataImpl(T& pose) const
  {
    TerrainResult result = OK;

    // get z
    double z;
    if (getHeight(pose.x(), pose.y(), z) == OK)
      pose.setZ(z);
    else
      result = NO_HEIGHT;

    // get roll and pitch
    l3::Vector3 normal;
    if (getNormal(pose, normal) == OK)
    {
      double roll;
      double pitch;
      double yaw = pose.yaw();
      l3::normalToRP(normal, yaw, roll, pitch);
      pose.setRPY(roll, pitch, yaw);
    }
    else
      result = static_cast<TerrainResult>(result | NO_NORMAL);

    return result;
  }

protected:
  TerrainResult getMaxHeightUnderFoot(const Foothold& foothold, double& height) const;

  std::map<FootIndex, Vector3> foot_size_map_;

  bool sample_max_height_under_foot_;
  unsigned int sampling_steps_x_;    // number of sampling steps in y
  unsigned int sampling_steps_y_;    // number of sampling steps in y

private:
  mutable Mutex model_lock_;
};
}  // namespace l3_footstep_planning

#endif
