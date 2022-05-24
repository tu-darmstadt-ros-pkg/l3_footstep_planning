//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, Filip Bjelonic, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_HFS_HEURISTIC_PLUGIN_H__
#define L3_FOOTSTEP_PLANNING_HFS_HEURISTIC_PLUGIN_H__

#include <ros/ros.h>

#include <l3_libs/types/types.h>

#include <l3_footstep_planning_plugins/base/footstep_planning_plugin.h>

namespace l3_footstep_planning
{
typedef std::pair<l3::Point, double> DiscreteHeuristicInformation;
typedef std::vector<DiscreteHeuristicInformation> DiscreteHeuristicInformationArray;
typedef std::map<FootIndex, DiscreteHeuristicInformationArray> DiscreteHeuristicInformationMap;

class HFSHeuristicPlugin : public virtual FootstepPlanningPlugin
{
public:
  typedef l3::SharedPtr<HFSHeuristicPlugin> Ptr;
  typedef l3::SharedPtr<const HFSHeuristicPlugin> ConstPtr;

  HFSHeuristicPlugin(const std::string& name);

  bool isUnique() const final { return false; }

  /**
   * @brief Gives you a Cost for how difficulty it will be to plan this foot index next (regarding the current step)
   * @param idx Index of the foot the cost is needed
   * @param step the current step
   * @param cost for planning the next feet at the current step
   * @param risk for how likely this footstep migh encounter a failure
   * @return true if there was no error, false if there is an error
   */
  virtual bool getFeetHeuristic(const l3::FootIndex idx, Step::ConstPtr step, double& cost, double& risk) const { return true; }
  /**
   * @brief Gives you a Cost for how difficulty it will be to plan the base next (regarding the current step)
   * @param idx Index of the base the cost is needed
   * @param step the current step
   * @param cost for planning the next feet at the current step
   * @param risk for how likely this footstep migh encounter a failure
   * @return true if there was no error, false if there is an error
   */
  virtual bool getBaseHeuristic(const l3::BaseIndex idx, Step::ConstPtr step, double& cost, double& risk) const { return true; }

  DiscreteHeuristicInformationArray getDiscreteHeuristicInformation(const FootIndex& idx) const
  {
    DiscreteHeuristicInformationMap::const_iterator itr = discrete_heuristic_information_.find(idx);
    if (itr == discrete_heuristic_information_.end())
      return DiscreteHeuristicInformationArray();
    else
      return itr->second;
  }

  VariantDataSet getVisualizationInformation() const { return vis_information_; }

protected:
  mutable DiscreteHeuristicInformationMap discrete_heuristic_information_;
  mutable VariantDataSet vis_information_;
};
}  // namespace l3_footstep_planning

#endif
