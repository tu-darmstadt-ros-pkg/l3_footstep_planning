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

#ifndef L3_PLANNING_FEEDBACK_VIS_H__
#define L3_PLANNING_FEEDBACK_VIS_H__

#include <l3_footstep_planning_vis_tools/base/planning_markers_vis_plugin.h>

namespace l3_footstep_planning
{
class PlanningFeedbackVis : public PlanningMarkersVisPlugin
{
public:
  PlanningFeedbackVis();

  void clear() override;

  void visualize(const msgs::PlanningFeedback& planning_feedback) override;

protected:
  void visualizeRecentlyVisitedSteps(const msgs::PlanningFeedback& planning_feedback);
  void visualizeTotalVisitedSteps(const msgs::PlanningFeedback& planning_feedback);

  void visualizeRecentlyVisitedBases(const msgs::PlanningFeedback& planning_feedback);
  void visualizeTotalVisitedBases(const msgs::PlanningFeedback& planning_feedback);

  struct StepDataMsgVisCompare
  {
    template <typename StepDataType>
    bool operator()(const StepDataType& lhs, const StepDataType& rhs) const
    {
      double lhs_a[6];
      toArray(lhs, lhs_a);
      double rhs_a[6];
      toArray(rhs, rhs_a);

      // we can't visualize orientation, thus we ignore it
      for (size_t i = 0; i < 6; i++)
      {
        if (lhs_a[i] < rhs_a[i])
          return true;
        else if (lhs_a[i] > rhs_a[i])
          return false;
      }
      return false;
    }

    template <typename StepDataType>
    void toArray(const StepDataType s, double (&a)[6]) const
    {
      a[0] = s.origin.pose.position.x;
      a[1] = s.origin.pose.position.y;
      a[2] = s.origin.pose.position.z;
      a[3] = s.target.pose.position.x;
      a[4] = s.target.pose.position.y;
      a[5] = s.target.pose.position.z;
    }
  };

  std::set<msgs::FootStepData, StepDataMsgVisCompare> visited_footholds_;
  std::set<msgs::BaseStepData, StepDataMsgVisCompare> visited_bases_;
};
}  // namespace l3_footstep_planning

#endif
