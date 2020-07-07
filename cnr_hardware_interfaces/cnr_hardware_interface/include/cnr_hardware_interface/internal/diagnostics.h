/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef  CNR_HARDWARE_INTERFACE_INTERNAL_DIAGNOSTICS_H
#define  CNR_HARDWARE_INTERFACE_INTERNAL_DIAGNOSTICS_H

#include <iostream>
#include <algorithm>
#include <chrono>  //NOLINT
#include <mutex>  //NOLINT
#include <cinttypes>
#include <csignal>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <realtime_utilities/circular_buffer.h>

class MainThreadSharedData
{
private:
  const int         windows_dim_;
  std::mutex        mtx_;
  double            cycle_time_;
  realtime_utilities::circ_buffer<double>   latency_msr_;
  realtime_utilities::circ_buffer<double>   cycle_time_msr_;
  realtime_utilities::circ_buffer<double>   calc_time_;
  realtime_utilities::circ_buffer<uint32_t> missed_cycles_;

public:
  explicit MainThreadSharedData(const int windows_dim)
    : windows_dim_(windows_dim)
    , cycle_time_(0)
    , latency_msr_(windows_dim)
    , cycle_time_msr_(windows_dim)
    , calc_time_(windows_dim)
    , missed_cycles_(windows_dim)
  {
  }

  int      getWindowDim()
  {
    return windows_dim_;
  }
  double   getCycleTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return cycle_time_;
  }

  double   getMeanActCycleTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::mean(cycle_time_msr_.get()) * 1e3;
  }
  double   getMeanCalcTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::mean(calc_time_.get()) * 1e3;
  }
  double   getMeanLatencyTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::mean(latency_msr_.get()) * 1e3;
  }
  uint32_t getMeanMissedCycles()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::mean(missed_cycles_.get());
  }

  double   getMaxActCycleTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::max(cycle_time_msr_.get()) * 1e3;
  }
  double   getMaxCalcTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::max(calc_time_.get()) * 1e3;
  }
  double   getMaxLatencyTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::max(latency_msr_.get()) * 1e3;
  }
  uint32_t getMaxMissedCycles()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::max(missed_cycles_.get());
  }

  double   getMinActCycleTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::min(cycle_time_msr_.get()) * 1e3;
  }
  double   getMinCalcTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::min(calc_time_.get()) * 1e3;
  }
  double   getMinLatencyTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::min(latency_msr_.get()) * 1e3;
  }
  uint32_t getMinMissedCycles()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::min(missed_cycles_.get());
  }

  void     setCycleTime(double    v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    cycle_time_    = v;
  }
  void     setActualCycleTime(double    v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    cycle_time_msr_.push_back(v);
  }
  void     setLatencyTime(double    v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    latency_msr_.push_back(v);
  }
  void     setCalcTime(double    v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    calc_time_.push_back(v);
  }
  void     setMissedCycles(uint32_t  v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    missed_cycles_.push_back(v);
  }
};

#endif  //  CNR_HARDWARE_INTERFACE_INTERNAL_DIAGNOSTICS_H
