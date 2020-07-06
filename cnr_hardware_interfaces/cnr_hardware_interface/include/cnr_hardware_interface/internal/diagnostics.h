#ifndef __CNR_HARDWARE_INTERFACE_DIAGNOSTICS__H__
#define __CNR_HARDWARE_INTERFACE_DIAGNOSTICS__H__

#ifndef __COE_DRIVER_UTILITES__H__
#define __COE_DRIVER_UTILITES__H__

#include <iostream>
#include <algorithm>
#include <chrono>
#include <mutex>
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
  MainThreadSharedData(const int windows_dim )
    : windows_dim_   (windows_dim)
    , cycle_time_    (0)
    , latency_msr_   (windows_dim)
    , cycle_time_msr_(windows_dim)
    , calc_time_     (windows_dim)
    , missed_cycles_ (windows_dim)
  {
  }

  int      getWindowDim        ( ) { return windows_dim_;   }
  double   getCycleTime        ( ) { std::lock_guard<std::mutex> lock(mtx_);  return cycle_time_;   }

  double   getMeanActCycleTime ( ) { std::lock_guard<std::mutex> lock(mtx_);  return realtime_utilities::mean( cycle_time_msr_.get() ) * 1e3;    }
  double   getMeanCalcTime     ( ) { std::lock_guard<std::mutex> lock(mtx_);  return realtime_utilities::mean( calc_time_.get() ) * 1e3;    }
  double   getMeanLatencyTime  ( ) { std::lock_guard<std::mutex> lock(mtx_);  return realtime_utilities::mean( latency_msr_.get() ) * 1e3;    }
  uint32_t getMeanMissedCycles ( ) { std::lock_guard<std::mutex> lock(mtx_);  return realtime_utilities::mean( missed_cycles_.get() );}

  double   getMaxActCycleTime  ( ) { std::lock_guard<std::mutex> lock(mtx_);  return realtime_utilities::max( cycle_time_msr_.get() ) * 1e3;    }
  double   getMaxCalcTime      ( ) { std::lock_guard<std::mutex> lock(mtx_);  return realtime_utilities::max( calc_time_.get() ) * 1e3;    }
  double   getMaxLatencyTime   ( ) { std::lock_guard<std::mutex> lock(mtx_);  return realtime_utilities::max( latency_msr_.get() ) * 1e3;    }
  uint32_t getMaxMissedCycles  ( ) { std::lock_guard<std::mutex> lock(mtx_);  return realtime_utilities::max( missed_cycles_.get() );}

  double   getMinActCycleTime  ( ) { std::lock_guard<std::mutex> lock(mtx_);  return realtime_utilities::min( cycle_time_msr_.get() ) * 1e3;    }
  double   getMinCalcTime      ( ) { std::lock_guard<std::mutex> lock(mtx_);  return realtime_utilities::min( calc_time_.get() ) * 1e3;    }
  double   getMinLatencyTime   ( ) { std::lock_guard<std::mutex> lock(mtx_);  return realtime_utilities::min( latency_msr_.get() ) * 1e3;    }
  uint32_t getMinMissedCycles  ( ) { std::lock_guard<std::mutex> lock(mtx_);  return realtime_utilities::min( missed_cycles_.get() );}

  void     setCycleTime        ( double    v ) { std::lock_guard<std::mutex> lock(mtx_);  cycle_time_    = v; }
  void     setActualCycleTime  ( double    v ) { std::lock_guard<std::mutex> lock(mtx_);  cycle_time_msr_.push_back( v ); }
  void     setLatencyTime      ( double    v ) { std::lock_guard<std::mutex> lock(mtx_);  latency_msr_.push_back( v ); }
  void     setCalcTime         ( double    v ) { std::lock_guard<std::mutex> lock(mtx_);  calc_time_.push_back( v ); }
  void     setMissedCycles     ( uint32_t  v ) { std::lock_guard<std::mutex> lock(mtx_);  missed_cycles_.push_back( v ); }

};





#endif


#endif
