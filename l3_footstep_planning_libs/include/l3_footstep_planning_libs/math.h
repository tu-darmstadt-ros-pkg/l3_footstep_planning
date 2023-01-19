//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
// Based on http://wiki.ros.org/footstep_planner by Johannes Garimort and Armin Hornung
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

#ifndef L3_FOOTSTEP_PLANNING_LIBS_MATH_H__
#define L3_FOOTSTEP_PLANNING_LIBS_MATH_H__

#define DEBUG_HASH 0
#define DEBUG_TIME 0

#include <math.h>

#include <l3_math/math.h>
#include <l3_math/angles.h>

#include <l3_libs/types/typedefs.h>

#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

namespace l3_footstep_planning
{
/// Used to scale continuous values in meter to discrete values in mm.
static const int cvMmScale = 1000;

/**
 * @return Squared euclidean distance between two integer coordinates
 * (cells).
 */
inline double euclideanDistanceSquared(int x1, int y1, int x2, int y2) { return l3::norm_sq(x1 - x2, y1 - y2); }

/// @return Squared euclidean distance between two coordinates.
inline double euclideanDistanceSquared(double x1, double y1, double x2, double y2) { return l3::norm_sq(x1 - x2, y1 - y2); }

inline double euclideanDistanceSquared(int x1, int y1, int z1, int x2, int y2, int z2) { return l3::norm_sq(x1 - x2, y1 - y2, z1 - z2); }

/// @return Squared euclidean distance between two coordinates.
inline double euclideanDistanceSquared(double x1, double y1, double z1, double x2, double y2, double z2) { return l3::norm_sq(x1 - x2, y1 - y2, z1 - z2); }

/// @return Euclidean distance between two integer coordinates (cells).
inline double euclideanDistance(int x1, int y1, int x2, int y2) { return sqrt(double(euclideanDistanceSquared(x1, y1, x2, y2))); }

/// @return Euclidean distance between two coordinates.
inline double euclideanDistance(double x1, double y1, double x2, double y2) { return sqrt(euclideanDistanceSquared(x1, y1, x2, y2)); }

/// @return Euclidean distance between two integer coordinates (cells).
inline double euclideanDistance(int x1, int y1, int z1, int x2, int y2, int z2) { return sqrt(double(euclideanDistanceSquared(x1, y1, z1, x2, y2, z2))); }

/// @return Euclidean distance between two coordinates.
inline double euclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2) { return sqrt(euclideanDistanceSquared(x1, y1, z1, x2, y2, z2)); }

inline double parabol(double x, double y, double a_inv, double b_inv) { return x * x * a_inv + y * y * b_inv; }

/// @return The distance of two neighbored cell.
inline double gridCost(int x1, int y1, int x2, int y2, float cell_size)
{
  int x = abs(x1 - x2);
  int y = abs(y1 - y2);

  if (x + y > 1)
    return M_SQRT2 * cell_size;
  else
    return cell_size;
}
}  // namespace l3_footstep_planning

#endif
