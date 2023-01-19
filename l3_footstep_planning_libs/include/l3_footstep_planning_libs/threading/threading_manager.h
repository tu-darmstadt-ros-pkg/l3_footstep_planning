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

#ifndef L3_FOOTSTEP_PLANNING_THREADING_MANAGER_H__
#define L3_FOOTSTEP_PLANNING_THREADING_MANAGER_H__

#include <ros/ros.h>

#include <thread>

#include <l3_libs/types/memory.h>

#include <l3_footstep_planning_libs/threading/queue.h>
#include <l3_footstep_planning_libs/threading/worker.h>

namespace l3_footstep_planning
{
namespace threading
{
template <typename JobType>
class ThreadingManager
{
public:
  // typedefs
  typedef l3::SharedPtr<ThreadingManager> Ptr;
  typedef l3::SharedPtr<const ThreadingManager> ConstPtr;

  ThreadingManager(int threads = -1, unsigned int jobs_per_thread = 100, bool auto_start = true)
  {
    // auto detect cores
    if (threads == -1)
      threads = static_cast<int>(std::thread::hardware_concurrency());

    ROS_INFO("[ThreadingManager] Spawning %i workers.", threads);
    for (int n = 0; n < threads; n++)
      workers_.push_back(l3::SharedPtr<Worker<JobType>>(new Worker<JobType>(queue_, jobs_per_thread, auto_start)));
  }

  virtual ~ThreadingManager()
  {
    ROS_INFO("[ThreadingManager] Destruct");
    stopJobs();
  }

  void addJob(l3::SharedPtr<JobType> job) { queue_.queueJob(job); }

  template<template <typename...> class Container>
  void addJobs(const Container<l3::SharedPtr<JobType>>& jobs) { queue_.queueJobs(jobs); }

  void stopJobs()
  {
    ROS_INFO("[ThreadingManager] Wait for thread termination...");
    for (typename std::list<l3::SharedPtr<Worker<JobType>>>::iterator itr = workers_.begin(); itr != workers_.end(); itr++)
      (*itr)->stop();
    deleteJobs();
    ROS_INFO("[ThreadingManager] Wait for thread termination...Done!");
  }

  void interruptJobs()
  {
    ROS_INFO("[ThreadingManager] Interrupt jobs...");
    for (typename std::list<l3::SharedPtr<Worker<JobType>>>::iterator itr = workers_.begin(); itr != workers_.end(); itr++)
      (*itr)->interruptJobs();
    deleteJobs();
    ROS_INFO("[ThreadingManager] Interrupt jobs...Done!");
  }

  void deleteJobs() { queue_.clear(); }

  bool hasJobsFinished() { return !queue_.hasOpenJobs(); }
  void waitUntilJobsFinished()
  {
    try
    {
      queue_.waitUntilJobsProcessed();
    }
    catch (boost::thread_interrupted& interrupt)
    {
      ROS_INFO("[ThreadingManager] Catched boost::interruption");
      interruptJobs();
      throw(interrupt);
    }
  }

protected:
  Queue<JobType> queue_;
  std::list<l3::SharedPtr<Worker<JobType>>> workers_;
};
}  // namespace threading
}  // namespace l3_footstep_planning

#endif
