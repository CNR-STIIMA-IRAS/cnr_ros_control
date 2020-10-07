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
#ifndef CNR_HARDWARE_INTERFACE_INTERNAL_CNR_ROBOT_HW_UTILS_H
#define CNR_HARDWARE_INTERFACE_INTERNAL_CNR_ROBOT_HW_UTILS_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <cnr_hardware_interface/internal/cnr_robot_hw_status.h>

namespace cnr_hardware_interface
{

// Iterator over enumr
typedef cnr_hardware_interface::EnumIterator< cnr_hardware_interface::tagStatusHw,
                                              cnr_hardware_interface::tagStatusHw::UNLOADED,
                                              cnr_hardware_interface::tagStatusHw::SRV_ERROR> StatusHwIterator;

inline std::string last_status_param(const std::string& hw_name)
{
  return "/" + hw_name + "/status/last_status";
}
inline std::string status_param(const std::string& hw_name)
{
  return "/" + hw_name + "/status/status";
}

inline
bool get_state(ros::NodeHandle& nh,
               const std::string& hw_name,
               cnr_hardware_interface::StatusHw& status,
               const ros::Duration& watchdog, std::string& error)
{
  std::string state;
  ros::Time st = ros::Time::now();
  error += " GET_STATE: nh: " + nh.getNamespace() + " param: " + last_status_param(hw_name);
  bool ok = false;
  do
  {
    if (nh.getParam(last_status_param(hw_name), state))
    {
      for (const StatusHw& it : StatusHwIterator())
      {
        if (state == to_string(it))
        {
          status = it;
          ok = true;
          break;
        }
      }
    }

    if ((watchdog.toSec() > 0) && (ros::Time::now() - st > watchdog))
    {
      error += " Timeout";
      break;
    }
  }
  while (ros::ok() && !ok);

  return ok;
}

inline
bool set_state(ros::NodeHandle& nh, const std::string& hw_name, const cnr_hardware_interface::StatusHw& status)
{
  nh.setParam(status_param(hw_name), cnr_hardware_interface::to_string(status));
  return true;
}


inline
bool get_state(ros::NodeHandle& nh,
               const std::vector<std::string> &hw_names,
               std::vector<cnr_hardware_interface::StatusHw>& status,
               std::string& error,
               const ros::Duration& watchdog)
{
  status.clear();
  for (auto const & hw_name : hw_names)
  {
    cnr_hardware_interface::StatusHw st;
    if (!get_state(nh, hw_name, st, watchdog, error))
    {
      error = "RobotHW (" + hw_name + ") Error Status";
      return false;
    }
    status.push_back(st);
  }
  return true;
}

}  // namespace cnr_hardware_interface

#endif  // CNR_HARDWARE_INTERFACE_INTERNAL_CNR_ROBOT_HW_UTILS_H
