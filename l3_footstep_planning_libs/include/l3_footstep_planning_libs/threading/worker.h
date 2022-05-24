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

#ifndef L3_FOOTSTEP_PLANNING_THREADING_WORKER_H__
#define L3_FOOTSTEP_PLANNING_THREADING_WORKER_H__

#include <ros/ros.h>

#include <boost/thread.hpp>

#include <l3_footstep_planning_libs/threading/queue.h>

namespace l3_footstep_planning
{
namespace threading
{
template <typename JobType>
class Worker
{
public:
  Worker(Queue<JobType>& queue, unsigned int jobs_per_thread = 10, bool auto_start = true)
    : queue_(queue)
    , jobs_per_thread_(jobs_per_thread)
    , exit_(false)
  {
    if (auto_start)
      start();
  }

  virtual ~Worker()
  {
    ROS_INFO_STREAM("[Worker (" << boost::this_thread::get_id() << ")] Destruct");
    stop();
  }

  void start()
  {
    ROS_INFO_STREAM("[Worker (" << boost::this_thread::get_id() << ")] Start request");
    exit_ = false;
    thread_ = boost::thread(&Worker::run, this, jobs_per_thread_);
  }

  void stop()
  {
    ROS_INFO_STREAM("[Worker (" << boost::this_thread::get_id() << ")] Stop request");

    // soft stop
    interruptJobs();
    exit_ = true;

    // hard stop
    thread_.interrupt();

    thread_.join();
  }

  void interruptJobs() { run_jobs_ = false; }

protected:
  void run(unsigned int n)
  {
    ROS_INFO_STREAM("[Worker (" << boost::this_thread::get_id() << ")] Started with " << n << " jobs per thread.");

    std::vector<l3::SharedPtr<JobType>> jobs;
    while (!exit_)
    {
      queue_.waitAndDequeueJobs(jobs, n);

      run_jobs_ = true;

      ROS_DEBUG_STREAM("[Worker (" << boost::this_thread::get_id() << ")] Deqeued " << jobs.size() << " jobs.");
      for (size_t i = 0; i < jobs.size() && run_jobs_; i++)
      {
        boost::this_thread::interruption_point();
        jobs[i]->run();
      }

      if (run_jobs_)
        queue_.justFinishedJobs(jobs.size());
    }
    ROS_INFO_STREAM("[Worker (" << boost::this_thread::get_id() << ")] Stopped!");
  }

  boost::thread thread_;

  Queue<JobType>& queue_;
  unsigned int jobs_per_thread_;
  bool exit_;
  std::atomic<bool> run_jobs_;
};
}  // namespace threading
}  // namespace l3_footstep_planning

#endif
