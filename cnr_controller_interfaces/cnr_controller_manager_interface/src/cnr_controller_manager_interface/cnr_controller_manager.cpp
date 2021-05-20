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
#include <algorithm>
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include <string>
#include <thread>
#include <realtime_utilities/diagnostics_interface.h>
#include <cnr_controller_interface_params/cnr_controller_interface_params.h>
#include <cnr_controller_manager_interface/cnr_controller_manager.h>

namespace cnr_controller_manager_interface
{

/**
 * 
 * 
 * 
 */
ControllerManager::ControllerManager(cnr_logger::TraceLogger* log,
                                     const std::string& hw_name,
                                     hardware_interface::RobotHW *robot_hw,
                                     const ros::NodeHandle& nh)
: ControllerManagerBase( log, hw_name ) , cm_( robot_hw, nh )
{
  
}

bool ControllerManager::loadController(const std::string& ctrl_to_load_name, const ros::Duration& watchdog)
{
  CNR_TRACE_START(logger_, "HW: " + getHwName() + ", CTRL: " + ctrl_to_load_name);
  if (!cm_.loadController(ctrl_to_load_name))
  {
    error_ = "ControllerManagerfailed while loading '" + ctrl_to_load_name + "'\n";
    controller_manager_msgs::ListControllerTypes ctrl_types;
    if (!listTypeRequest(ctrl_types, error_, watchdog))
    {
      CNR_RETURN_FALSE(logger_, "HW: " + getHwName() + ", CTRL: " + ctrl_to_load_name);
    }
    error_ += "Available " + std::to_string((int)(ctrl_types.response.types.size())) +  "# classes:\n";
    for (auto const & t : ctrl_types.response.types)
    {
      error_ += "-" + t + "\n" ;
    }
    CNR_RETURN_FALSE(logger_, "HW: " + getHwName() + ", CTRL: " + ctrl_to_load_name);
  }

  const std::string n = cnr::control::ctrl_list_param_name(getHwName());
  std::vector<std::string> l;
  if (ros::param::has(n))
  {
    ros::param::get(n, l);
  }
  if (std::find(l.begin(), l.end(), ctrl_to_load_name) == l.end())
  {
    l.push_back(ctrl_to_load_name);
    ros::param::set(n, l);
  }

  controllers_.emplace( ctrl_to_load_name, cm_.getControllerByName(ctrl_to_load_name) );
  
  CNR_RETURN_TRUE(logger_, "HW: " + getHwName() + ", CTRL: " + ctrl_to_load_name);
}

bool ControllerManager::switchController(const std::vector<std::string>&  to_start_names,
                                         const std::vector<std::string>&  to_stop_names,
                                         const int                        strictness, 
                                         const ros::Duration&             watchdog)
{
  CNR_TRACE_START(logger_, "HW: "+ getHwName());

  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;

  //--
  if (to_start_names.size())
  {
    CNR_DEBUG(logger_, to_string(to_start_names, "to_start_names  "));
    for (auto const & ctrl : to_start_names)
      start_controllers.push_back(ctrl);
  }
  if (to_stop_names.size())
  {
    CNR_DEBUG(logger_, to_string(to_stop_names, "to_stop_names "));
    for (auto const & ctrl : to_stop_names)
      stop_controllers.push_back(ctrl);
  }

  // switch controllers
  if ((start_controllers.size() == 0) && (stop_controllers.size() == 0))
  {
    CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
  }

  if (!cm_.switchController(start_controllers, stop_controllers, strictness) )
  {
    error_ = "The ControllerManager failed in switchin the controller. Abort.";
    CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
  }

  if (watchdog.toSec() > 0)
  {
    // check the param, and exit only if the state is running!
    for (const std::string ctrl_name : start_controllers)
    {
      if (!cnr::control::ctrl_check_state(getNamespace(), ctrl_name, "RUNNING", error_, watchdog))
      {
        CNR_RETURN_FALSE(logger_, "HW: " + getHwName()+": " + error_);
      }
    }
    for (const std::string ctrl_name : stop_controllers)
    {
      if (!cnr::control::ctrl_check_state(getNamespace(), ctrl_name, "STOPPED", error_, watchdog))
      {
        CNR_RETURN_FALSE(logger_, "HW: " + getHwName()+ ": " + error_);
      }
    }
  }
  CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
}

/**
 * @brief ControllerManager::unloadController
 * @param ctrl_to_unload_name
 * @param watchdog
 * @return
 */
bool ControllerManager::unloadController(const std::string& ctrl_to_unload_name, const ros::Duration& watchdog)
{
  bool ret = true;
  CNR_TRACE_START(logger_, "HW: "+ getHwName()+", CTRL: " + ctrl_to_unload_name);

  if (!cm_.unloadController( ctrl_to_unload_name ))
  {
    error_ = "The ControllerManager failed in unloading the controller '"+ctrl_to_unload_name+ "'. Abort.";
    ret = false;
    //CNR_RETURN_FALSE(logger_, "HW: "+ getHwName()+", CTRL: " + ctrl_to_unload_name);
  }

  std::string st = (ret  ? "UNLOADED" : "ERROR_UNLOAD");
  std::vector<std::string> status_history;
  ros::param::get(cnr::control::ctrl_status_param_name(getHwName(), ctrl_to_unload_name),  status_history);

  status_history.push_back(st);
  ros::param::set(cnr::control::ctrl_status_param_name(getHwName(), ctrl_to_unload_name),  status_history);
  ros::param::set(cnr::control::ctrl_last_status_param_name(getHwName(), ctrl_to_unload_name), st) ;

  if( ret )
  {
    if(controllers_.find(ctrl_to_unload_name) != controllers_.end())
    {
      controllers_.erase( controllers_.find(ctrl_to_unload_name) );
    }
  }

  CNR_RETURN_BOOL(logger_, ret, "HW: "+ getHwName()+", CTRL: " + ctrl_to_unload_name);
}


/**
 * @brief ControllerManager::diagnostics
 * @param stat
 */
void ControllerManager::diagnosticsInfo(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for ( const auto & ctrl : controllers_)
  {
    realtime_utilities::DiagnosticsInterface* cnr_ctrl =
                                    dynamic_cast<realtime_utilities::DiagnosticsInterface*>(ctrl.second);
    if(cnr_ctrl)
    {
      cnr_ctrl->diagnosticsInfo(stat);
    }
  }
}


/**
 * @brief ControllerManager::diagnostics
 * @param stat
 */
void ControllerManager::diagnosticsWarn(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for ( const auto & ctrl : controllers_)
  {
    realtime_utilities::DiagnosticsInterface* cnr_ctrl =
                                    dynamic_cast<realtime_utilities::DiagnosticsInterface*>(ctrl.second);
    if( cnr_ctrl )
    {
      cnr_ctrl->diagnosticsWarn(stat);
    }
  }
}


/**
 * @brief ControllerManager::diagnostics
 * @param stat
 */
void ControllerManager::diagnosticsError(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for ( const auto & ctrl : controllers_)
  {
    realtime_utilities::DiagnosticsInterface* cnr_ctrl =
                                    dynamic_cast<realtime_utilities::DiagnosticsInterface*>(ctrl.second);
    if( cnr_ctrl )
    {
      cnr_ctrl->diagnosticsError(stat);
    }
  }
}

void ControllerManager::diagnosticsPerformance(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for ( const auto & ctrl : controllers_)
  {
    realtime_utilities::DiagnosticsInterface* cnr_ctrl =
                                    dynamic_cast<realtime_utilities::DiagnosticsInterface*>(ctrl.second);
    if( cnr_ctrl )
    {
      cnr_ctrl->diagnosticsPerformance(stat);
    }
  }
}



}  // namespace cnr_controller_manager_interface
