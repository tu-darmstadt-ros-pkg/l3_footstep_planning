//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_LIBS_STEP_PLAN_H__
#define L3_FOOTSTEP_PLANNING_LIBS_STEP_PLAN_H__

#include <ros/ros.h>

#include <tf/tf.h>

#include <boost/thread/shared_mutex.hpp>

#include <l3_libs/types/step_queue.h>

#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

namespace l3_footstep_planning
{
using namespace l3;

/**
 * Wrapper class for l3_footstep_planning_msgs::StepPlan to provide
 * advanced operations.
 */
class StepPlan
{
public:
  // typedefs
  typedef l3::SharedPtr<StepPlan> Ptr;
  typedef l3::SharedPtr<const StepPlan> ConstPtr;

  StepPlan();
  StepPlan(const StepPlan& other);
  StepPlan(StepPlan&& other);
  StepPlan(const msgs::StepPlan& msg) { fromMsg(msg); }

protected:
  StepPlan(const StepPlan& other, SharedLock /*other_lock*/);
  StepPlan(StepPlan&& other, UniqueLock /*other_lock*/);

public:
  StepPlan& operator=(const StepPlan& step_plan);
  StepPlan& operator=(StepPlan&& step_plan);

  StepPlan& operator=(const msgs::StepPlan& step_plan);
  StepPlan& operator+(const msgs::StepPlan& step_plan);
  StepPlan& operator|(const msgs::StepPlan& step_plan);

  StepPlan& operator+(const msgs::Step& step);
  StepPlan& operator|(const msgs::Step& step);
  StepPlan& operator-(const msgs::Step& step);

  msgs::ErrorStatus fromMsg(const msgs::StepPlan& step_plan);
  msgs::ErrorStatus toMsg(msgs::StepPlan& step_plan) const;
  msgs::StepPlan toMsg() const;

  std::string toString() const;

  void clear();

  inline bool empty() const { return steps_.empty(); }

  inline size_t size() const { return steps_.size(); }

  // getter and setter
  inline const std_msgs::Header& getHeader() const { return header_; }
  inline std_msgs::Header& getHeader() { return header_; }
  inline const FootholdArray& getStart() const { return start_; }
  inline FootholdArray& getStart() { return start_; }
  inline const FootholdArray& getGoal() const { return goal_; }
  inline FootholdArray& getGoal() { return goal_; }
  inline const StepQueue& getSteps() const { return steps_; }
  inline StepQueue& getSteps() { return steps_; }

  // helper
  inline StepIndex getFirstStepIndex() const { return steps_.firstStepIndex(); }
  inline StepIndex getLastStepIndex() const { return steps_.lastStepIndex(); }

  msgs::ErrorStatus insertStep(Step::Ptr step);
  inline msgs::ErrorStatus insertStep(const l3_msgs::Step& step) { return insertStep(Step::Ptr(new Step(step))); }

  inline bool hasStep(StepIndex step_idx) const { return steps_.hasStep(step_idx); }

  static bool getStep(const msgs::StepPlan& step_plan, const StepIndex& step_idx, msgs::Step& step);
  inline bool getStep(const StepIndex& step_idx, Step::Ptr& step) const { return steps_.getStep(step_idx, step); }
  bool getStep(const StepIndex& step_idx, Step& step) const;
  inline Step::Ptr getStep(const StepIndex& step_idx) const { return steps_.getStep(step_idx); }

  /**
   * @brief Search fors step data to be executeded at specific dt relative to
   * given step index.
   * @param step_idx Reference step index for time
   * @param dt delta time to look forward
   * @param closing_step_size How many steps from the end of the queue should be ignored as they
   * belong to the closing step
   * @return pointer to corresponding step, otherwise nullptr
   */
  Step::Ptr getStep(const StepIndex& step_idx, double dt, unsigned int closing_step_size = 0u) const;

  const Step::Ptr getStepAt(unsigned int position) const;
  inline Step::Ptr getfirstStep() const { return steps_.firstStep(); }
  inline Step::Ptr getLastStep() const { return steps_.lastStep(); }

  inline const Step::Ptr popStep() { return steps_.pop(); }

  inline void removeStep(StepIndex step_idx) { steps_.removeStep(step_idx); }

  void removeStepAt(unsigned int position);

  inline void removeSteps(StepIndex from_step_idx, StepIndex to_step_idx = -1) { steps_.removeSteps(from_step_idx, to_step_idx); }

  /**
   * @brief Appends a step plan to current step plan. StepIndeces of the input step plan are adapted to seamlessly
   * append to exisiting steps. No transformation will be performed!
   * @param step_plan Step plan to be merged into current step plan.
   * @return error status
   */
  inline msgs::ErrorStatus appendStepPlan(const msgs::StepPlan& step_plan) { return appendStepPlan(StepPlan(step_plan)); }
  msgs::ErrorStatus appendStepPlan(const StepPlan& step_plan);

  /**
   * @brief Merges the given step plan to current step plan. No transformation will be performed!
   * @param step_plan Step plan to be merged into current step plan.
   * Caution: Very unrestrictive for input step_plan, does not perform consisty checks!
   * @return error status
   */
  inline msgs::ErrorStatus updateStepPlan(const msgs::StepPlan& step_plan) { return updateStepPlan(StepPlan(step_plan)); }
  msgs::ErrorStatus updateStepPlan(const StepPlan& step_plan);

  /**
   * @brief Stitches the given step plan into the current step plan starting at step_idx. Hereby the all steps
   * in range [0, step_idx] are kept and all steps in range (step_idx, end] are taken from the new input step plan.
   * The overlapping step at step_idx from both step plans are taken as reference points (= both are
   * representing the equal position in their local frame) to transform the input step plan in order
   * to stitch seamlessly both plans together.
   * @param step_plan Step plan to be merged into current step plan.
   * @param step_idx Stitching point where the input step plan will be stitched to the current step plan (set 0 for auto)
   * @return error status
   */
  inline msgs::ErrorStatus stitchStepPlan(const msgs::StepPlan& step_plan, const StepIndex& step_idx = 0) { return stitchStepPlan(StepPlan(step_plan), step_idx); }
  msgs::ErrorStatus stitchStepPlan(const StepPlan& step_plan, const StepIndex& step_idx = 0);

  /**
   * @brief Transform the step plan using the given transform.
   * @param step_plan Input step plan to apply the transform
   * @param transform Transform to applay
   * @param header (optional) New header information
   * @return Transformed step plan
   */
  inline static msgs::StepPlan transformStepPlan(const msgs::StepPlan& step_plan, const tf::Transform& transform, const std_msgs::Header& header = std_msgs::Header())
  {
    return transformStepPlan(step_plan, Transform(transform), header);
  }
  static msgs::StepPlan transformStepPlan(const msgs::StepPlan& step_plan, const Transform& transform, const std_msgs::Header& header = std_msgs::Header());
  inline StepPlan& transform(const tf::Transform transform, const std_msgs::Header& header = std_msgs::Header()) { return this->transform(Transform(transform), header); }
  StepPlan& transform(const Transform& transform, const std_msgs::Header& header = std_msgs::Header());

protected:
  /** mutex free versions */

  /**
   * @brief Inserts step into step plan. If the step does already exists it will be
   * overwritten and *not tagged* as modified.
   * @param step Step to be inserted
   * @return error status
   */
  msgs::ErrorStatus _insertStep(Step::Ptr step);

  /**
   * @brief Appends a step plan to current step plan. No transformation will be done!
   * @param step_plan Step plan to be merged into current step plan.
   * @return error status
   */
  msgs::ErrorStatus _appendStepPlan(const StepPlan& step_plan);

  /**
   * @brief Merges the given step plan to current step plan.
   * Already exisiting steps *will* be tagged as modified. No transformation will be done!
   * Caution: Very unrestrictive for input step_plan, does not perform consisty checks!
   * @param step_plan Step plan to be merged into current step plan.
   * @return error status
   */
  msgs::ErrorStatus _updateStepPlan(const StepPlan& step_plan);

  /**
   * @brief Stitches the given step plan into the current step plan starting at step_idx. Hereby the all
   * in range [0, step_idx] are kept and all steps in range (step_idx, inf] are taken from input step plan.
   * The steps at step_idx from both step plans are taken as reference points (= both are
   * representing the equal position in world frame) to transform the input step plan towards each other.
   * Finally, all steps are stitched to the current step plan based on the determined
   * transformation.
   * @param step_plan Step plan to be merged into current step plan.
   * @param step_idx Leading step index from which to start merging.
   * @return error status
   */
  msgs::ErrorStatus _stitchStepPlan(const StepPlan& step_plan, StepIndex step_idx = 0);

  mutable Mutex step_plan_mutex_;

  std_msgs::Header header_;
  FootholdArray start_;
  FootholdArray goal_;
  StepQueue steps_;
};

inline std::ostream& operator<<(std::ostream& stream, const StepPlan& step_plan) { return stream << step_plan.toString(); }
}  // namespace l3_footstep_planning

#endif
