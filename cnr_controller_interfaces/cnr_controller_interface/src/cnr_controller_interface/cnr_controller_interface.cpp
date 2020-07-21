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
#include <cnr_controller_interface/cnr_controller_interface.h>

namespace cnr_controller_interface
{



std::vector<std::string> get_names(const std::vector< controller_manager_msgs::ControllerState >& controllers)

{
  std::vector<std::string> ret;
  for (auto & ctrl : controllers) ret.push_back(ctrl.name);
  return ret;
}
//============


//============ FUNCTIONS TO DEFINE THE PARAMETERS WHERE THE CTRL STATUS IS LOADED
std::string ctrl_list_param(const std::string& hw_name)
{
  return "/" + hw_name + "/status/controllers_list";
}
std::string last_status_param(const std::string& hw_name, const std::string& ctrl_name)
{
  return "/" + hw_name + "/status/controllers/" + ctrl_name + "/last_status";
}

std::string status_param(const std::string& hw_name, const std::string& ctrl_name)
{
  return "/" + hw_name + "/status/controllers/" + ctrl_name + "/status";
}
bool get_state(const std::string& hw_name,
               const std::string& ctrl_name,
               std::string& status,
               std::string& error,
               const ros::Duration& watchdog)
{
  ros::Time st = ros::Time::now();
  const std::string p = last_status_param(hw_name, ctrl_name);
  while (ros::ok())
  {
    if (ros::param::get(p, status))
    {
      break;
    }

    if ((watchdog.toSec() > 0) && (ros::Time::now() > (st + watchdog)))
    {
      error += "Timeout. Param " + p + " does not exist";
      return false;
    }
    ros::Duration(0.001).sleep();
  }
  return true;
}
//============





}
