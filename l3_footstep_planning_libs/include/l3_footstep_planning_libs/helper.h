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

#ifndef L3_FOOTSTEP_PLANNING_LIBS_HELPER_H__
#define L3_FOOTSTEP_PLANNING_LIBS_HELPER_H__

#include <fstream>

#include <ros/ros.h>
#include <tf/tf.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <nav_msgs/OccupancyGrid.h>

#include <vigir_generic_params/generic_params_msgs.h>

#include <l3_libs/types/types.h>
#include <l3_libs/helper.h>

#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

#include <l3_footstep_planning_libs/typedefs.h>
#include <l3_footstep_planning_libs/modeling/state.h>

namespace l3_footstep_planning
{
using namespace l3;

/**
 * Helper to set quickly finish state of action server based on result
 */
template <typename ActionSpec, typename ActionResult>
void actionServerFinished(SimpleActionServer<ActionSpec>& as, const ActionResult& result)
{
  if (hasError(result.status))
    as.setAborted(result, toString(result.status));
  else
    as.setSucceeded(result, toString(result.status));
}

// loading of common parameters
bool getXYZ(ros::NodeHandle& nh, const std::string name, geometry_msgs::Vector3& val);
bool getRPY(ros::NodeHandle& nh, const std::string name, geometry_msgs::Vector3& val);

// helper functions
bool getGridMapCoords(const nav_msgs::OccupancyGrid& map, double x, double y, int& map_x, int& map_y);
bool getGridMapIndex(const nav_msgs::OccupancyGrid& map, double x, double y, int& idx);

// transforms arbitrary multi array into a csv table
template <typename T>
bool toCSV(const std::string file, const std::vector<std::vector<T>>& data, bool append = false)
{
  std::ofstream outfile;
  if (append)
    outfile.open(file, std::ios_base::app);
  else
    outfile.open(file);

  if (!outfile.is_open())
  {
    ROS_ERROR("Could not write to '%s'!", file.c_str());
    return false;
  }

  for (const typename std::vector<T>& v : data)
  {
    for (size_t i = 0; i < v.size(); i++)
    {
      outfile << v[i];

      if (i < (v.size() - 1))
        outfile << ",";
    }

    outfile << "\n";
  }

  outfile.close();

  return true;
}

FootIndexArray applyFootIdxWhitelist(const FootIndexArray& foot_idx, const FootIndexSet& foot_idx_whitelist);
l3_msgs::FootholdArray applyFootIdxWhitelist(const l3_msgs::FootholdArray& footholds, const FootIndexSet& foot_idx_whitelist);
FootholdArray applyFootIdxWhitelist(const FootholdArray& footholds, const FootIndexSet& foot_idx_whitelist);
FootholdPtrArray applyFootIdxWhitelist(const FootholdPtrArray& footholds, const FootIndexSet& foot_idx_whitelist);
FootholdConstPtrArray applyFootIdxWhitelist(const FootholdConstPtrArray& footholds, const FootIndexSet& foot_idx_whitelist);

Pose discretize(const Pose& pose, const DiscreteResolution& resolution);

inline Foothold discretize(const Foothold& foothold, const DiscreteResolution& resolution)
{
  return Foothold(foothold.idx, discretize(foothold.pose(), resolution), foothold.header, foothold.data);
}

inline FloatingBase discretize(const FloatingBase& floating_base, const DiscreteResolution& resolution)
{
  return FloatingBase(floating_base.idx, discretize(floating_base.pose(), resolution), floating_base.header, floating_base.data);
}

FootholdPtrArray getNeutralStance(const FloatingBase& floating_base);
FootholdPtrArray getNeutralStance(const FloatingBase& floating_base, const DiscreteResolution& resolution);

FootholdPtrArray expandNeutralStance(FloatingBase::ConstPtr floating_base, State::ConstPtr ref_state, const DiscreteResolution& resolution);
}  // namespace l3_footstep_planning

#endif
