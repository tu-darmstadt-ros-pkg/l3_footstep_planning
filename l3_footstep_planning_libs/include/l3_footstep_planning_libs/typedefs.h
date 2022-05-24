//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_TYPEDEFS_H__
#define L3_FOOTSTEP_PLANNING_TYPEDEFS_H__

#include <eigen3/Eigen/Eigen>
#include <boost/shared_ptr.hpp>

#include <vigir_generic_params/parameter_set.h>

#include <l3_libs/types/types.h>
#include <l3_footstep_planning_libs/math.h>

namespace l3_footstep_planning
{
typedef unsigned long int UID;

typedef std::size_t Hash;

struct Resolution
{
  Resolution()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , angle(0.0)
  {}

  /**
   * @brief Constructor for Resolution
   * @param x cell size in x
   * @param y cell size in y
   * @param z cell size in z
   * @param angle angular size for roll, pitch and yaw
   */
  Resolution(double x, double y, double z, double angle)
    : x(x)
    , y(y)
    , z(z)
    , angle(angle)
  {}

  Resolution invert() const { return Resolution(1.0 / x, 1.0 / y, 1.0 / z, 1.0 / angle); }

  double x, y, z, angle;
};

class DiscreteResolution
{
public:
  DiscreteResolution()
    : to_cont_()
    , to_disc_()
  {}

  /**
   * @brief Constructor for PlanningResolution
   * @param x cell size in x
   * @param y cell size in y
   * @param z cell size in z
   * @param angle angular size for roll, pitch and yaw
   */
  DiscreteResolution(double x, double y, double z, double angle)
    : num_angle_bins_(static_cast<unsigned int>(round(2.0 * M_PI / angle)))
    , to_cont_(x, y, z, angle)
    , to_disc_(to_cont_.invert())
  {}

  /**
   * @brief Constructor for PlanningResolution
   * @param x cell size in x
   * @param y cell size in y
   * @param z cell size in z
   * @param num_angle_bins number of angle bins to use
   */
  DiscreteResolution(double x, double y, double z, unsigned int num_angle_bins)
    : num_angle_bins_(num_angle_bins)
    , to_cont_(x, y, z, 2.0 * M_PI / static_cast<double>(num_angle_bins_))
    , to_disc_(to_cont_.invert())
  {}
  DiscreteResolution(double x, double y, double z, int num_angle_bins)
    : DiscreteResolution(x, y, z, static_cast<unsigned int>(num_angle_bins))
  {}

  DiscreteResolution(const XmlRpc::XmlRpcValue& p)
    : DiscreteResolution(vigir_generic_params::ParameterSet(p))
  {}

  DiscreteResolution(const vigir_generic_params::ParameterSet& p)
  {
    double angle_bin_size;
    if (p.hasParam("num_angle_bins"))
    {
      num_angle_bins_ = p.param("num_angle_bins", 72);
      angle_bin_size = 2.0 * M_PI / static_cast<double>(num_angle_bins_);
    }
    else if (p.hasParam("yaw"))
    {
      ROS_WARN("Deprecation warning: 'yaw' will not be supported in future versions. Please use 'angle' from now.");
      angle_bin_size = p.param("yaw", 2.0 * M_PI / 72.0);
      num_angle_bins_ = static_cast<unsigned int>(round(2.0 * M_PI / angle_bin_size));
    }
    else if (p.hasParam("angle"))
    {
      angle_bin_size = p.param("angle", 2.0 * M_PI / 72.0);
      num_angle_bins_ = static_cast<unsigned int>(round(2.0 * M_PI / angle_bin_size));
    }
    else
      ROS_ERROR("[DiscreteResolution] Could not find any angular resolution. Add either 'num_angle_bins' or 'angle' information to your resolution.");

    to_cont_ = Resolution(p.param("x", 0.1), p.param("y", 0.1), p.param("z", 0.1), angle_bin_size);
    to_disc_ = to_cont_.invert();
  }

  inline const unsigned int numAngleBins() const { return num_angle_bins_; }

  inline const Resolution& toCont() const { return to_cont_; }

  inline const Resolution& toDiscrete() const { return to_disc_; }

  inline double toContX(int x) const { return cont_val(x, to_cont_.x); }
  inline double toContY(int y) const { return cont_val(y, to_cont_.y); }
  inline double toContZ(int z) const { return cont_val(z, to_cont_.z); }
  inline double toContAngle(int angle) const { return cont_val(angle, to_cont_.angle); }

  inline int toDiscX(double x) const { return disc_val(x, to_disc_.x); }
  inline int toDiscY(double y) const { return disc_val(y, to_disc_.y); }
  inline int toDiscZ(double z) const { return disc_val(z, to_disc_.z); }
  inline int toDiscAngle(double angle) const { return disc_val(angle, to_disc_.angle); }

private:
  inline double cont_val(int val, double to_cont) const { return static_cast<double>(val) * to_cont; }

  inline int disc_val(double val, double to_disc) const { return static_cast<int>(round(val * to_disc)); }

  unsigned int num_angle_bins_;
  Resolution to_cont_;
  Resolution to_disc_;
};

// assertion to ensure that alle classes are implemented to support movable operation
L3_STATIC_ASSERT_MOVEABLE(UID)
L3_STATIC_ASSERT_MOVEABLE(Hash)
L3_STATIC_ASSERT_MOVEABLE(Resolution)
L3_STATIC_ASSERT_MOVEABLE(DiscreteResolution)
}  // namespace l3_footstep_planning

#endif
