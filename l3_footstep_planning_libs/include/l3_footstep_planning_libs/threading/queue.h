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

#ifndef L3_FOOTSTEP_PLANNING_THREADING_QUEUE_H__
#define L3_FOOTSTEP_PLANNING_THREADING_QUEUE_H__

#include <ros/ros.h>

#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

namespace l3_footstep_planning
{
namespace threading
{
template <typename JobType>
class Queue
{
public:
  Queue()
    : job_counter_(0)
  {}

  void clear()
  {
    boost::mutex::scoped_lock lock(queued_jobs_mutex_);
    queued_jobs_ = typename std::queue<l3::SharedPtr<JobType>>();
    job_counter_ = 0;
    jobs_finished_condition_.notify_all();
  }

  void queueJob(l3::SharedPtr<JobType> job)
  {
    {
      // scope needed so mutex gets unlocked when other threads are notified
      boost::mutex::scoped_lock lock(queued_jobs_mutex_);
      queued_jobs_.push(job);
    }
    queued_jobs_condition_.notify_one();
  }

  template<template <typename...> class Container>
  void queueJobs(const Container<l3::SharedPtr<JobType>>& jobs)
  {
    {
      // scope needed so mutex gets unlocked when other threads are notified
      boost::mutex::scoped_lock lock(queued_jobs_mutex_);
      for (l3::SharedPtr<JobType> j : jobs)
        queued_jobs_.push(j);
    }
    queued_jobs_condition_.notify_all();
  }

  l3::SharedPtr<JobType> waitAndDequeueJob()
  {
    boost::mutex::scoped_lock lock(queued_jobs_mutex_);

    if (queued_jobs_.empty())
    {
      ROS_DEBUG("[Queue] Waiting for Job getting queued...");
      queued_jobs_condition_.wait(lock);
    }

    ROS_DEBUG("[Queue] Pop Job.");

    l3::SharedPtr<JobType> front = queued_jobs_.front();
    queued_jobs_.pop();
    job_counter_++;

    return front;
  }

  void waitAndDequeueJobs(std::vector<l3::SharedPtr<JobType>>& jobs, unsigned int n)
  {
    boost::mutex::scoped_lock lock(queued_jobs_mutex_);

    if (queued_jobs_.empty())
    {
      ROS_DEBUG("[Queue] Waiting for Job getting queued...");
      queued_jobs_condition_.wait(lock);
    }

    ROS_DEBUG("[Queue] Pop %u Job(s).", n);
    n = std::min(n, static_cast<unsigned int>(queued_jobs_.size()));
    jobs.resize(n);

    for (unsigned int i = 0; i < n; i++)
    {
      jobs[i] = queued_jobs_.front();
      queued_jobs_.pop();
      job_counter_++;
    }
  }

  void justFinishedJobs(unsigned int n)
  {
    boost::mutex::scoped_lock lock(queued_jobs_mutex_);

    if (job_counter_ > n)
      job_counter_ -= n;
    else
      job_counter_ = 0;

    if (queued_jobs_.empty() && job_counter_ == 0)
    {
      ROS_DEBUG("[Queue] Notify for finished job.");
      jobs_finished_condition_.notify_all();
    }
  }

  bool hasOpenJobs() const
  {
    boost::mutex::scoped_lock lock(queued_jobs_mutex_);
    return !queued_jobs_.empty() || job_counter_ != 0;
  }

  void waitUntilJobsProcessed() const
  {
    boost::mutex::scoped_lock lock(queued_jobs_mutex_);

    if (!queued_jobs_.empty())
    {
      ROS_DEBUG("[Queue] Waiting for Jobs getting finished...");
      jobs_finished_condition_.wait(lock);
      ROS_DEBUG("[Queue] Waiting for Jobs getting finished...Done!");
    }
  }

protected:
  std::queue<l3::SharedPtr<JobType>> queued_jobs_;
  unsigned int job_counter_;

  mutable boost::mutex queued_jobs_mutex_;

  mutable boost::condition_variable queued_jobs_condition_;
  mutable boost::condition_variable jobs_finished_condition_;
};
}  // namespace threading
}  // namespace l3_footstep_planning

#endif
