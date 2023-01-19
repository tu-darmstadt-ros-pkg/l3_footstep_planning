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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_STEP_RANGE_H__
#define L3_FOOTSTEP_PLANNING_PLUGINS_STEP_RANGE_H__

#include <ros/ros.h>

#include <l3_libs/types/types.h>

#include <l3_footstep_planning_libs/typedefs.h>

namespace l3_footstep_planning
{
struct StepRangePolygon
{
  typedef std::pair<int, int> Point;
  typedef std::vector<Point> Points;

  StepRangePolygon();
  ~StepRangePolygon();

  bool fromYaml(XmlRpc::XmlRpcValue& p, const DiscreteResolution& res);

  bool pointWithinPolygon(int x, int y) const;
  inline bool pointWithinPolygon(double x, double y) const { return pointWithinPolygon(res_.toDiscX(x), res_.toDiscY(y)); }

  std::string toString() const;

  int min_x, min_y, min_roll, min_pitch, min_yaw;
  int max_x, max_y, max_roll, max_pitch, max_yaw;

  double max_step_range_width;
  double max_step_range_width_sq;

  Points points;

protected:
  bool parseAngles(XmlRpc::XmlRpcValue& p, const std::string& key, const DiscreteResolution& res, int& min_angle, int& max_angle);
  bool parseEdges(XmlRpc::XmlRpcValue& p, const DiscreteResolution& res);

  bool generateRechabilityMap();

  DiscreteResolution res_;

  // array for quick checks of pointWithinPolygon
  l3::Array2D<bool> step_range_;
};

typedef std::pair<const l3::FootIndex, StepRangePolygon> FootStepRangePair;
typedef std::map<l3::FootIndex, StepRangePolygon> FootStepRangeMap;

typedef std::pair<const l3::BaseIndex, StepRangePolygon> FloatingBaseStepRangePair;
typedef std::map<l3::BaseIndex, StepRangePolygon> FloatingBaseStepRangeMap;

inline std::ostream& operator<<(std::ostream& stream, const StepRangePolygon& polygon) { return stream << polygon.toString(); }
}  // namespace l3_footstep_planning

#endif
