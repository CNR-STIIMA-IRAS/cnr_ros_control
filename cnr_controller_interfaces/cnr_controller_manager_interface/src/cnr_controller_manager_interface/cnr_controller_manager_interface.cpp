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
#include <cnr_controller_manager_interface/cnr_controller_manager_interface.h>

namespace cnr_controller_manager_interface
{

/**
 * 
 * 
 * 
 */
ControllerManagerInterface::ControllerManagerInterface(const cnr_logger::TraceLoggerPtr& log,
                                     const std::string& hw_name,
                                     controller_manager::ControllerManager* cm)
: ControllerManagerInterfaceBase( log, hw_name ), cm_(cm)
{
  
}

ControllerManagerInterface::~ControllerManagerInterface()
{
  try
  {
    if(!stopUnloadAllControllers())
    {
      CNR_FATAL(logger_, "HW: " << getHwName() << ",  Error in stopping the controllers!");
    }
    for( auto & ctrl : controllers_)
    {
      ctrl.second = nullptr;
    }
    controllers_.clear();
    cm_ = nullptr;
  }
  catch(const std::exception& e)
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << "Exception: " << std::endl;
    std::cerr << e.what() << std::endl;
  }
}

bool ControllerManagerInterface::loadController(const std::string& ctrl_to_load_name, const ros::Duration& watchdog)
{
  CNR_TRACE_START(logger_, "HW: " + getHwName() + ", CTRL: " + ctrl_to_load_name);
  if(controllers_.find(ctrl_to_load_name)!=controllers_.end())
  {
    CNR_WARN(logger_, "HW: " + getHwName() + ",  The CTRL '" + ctrl_to_load_name + "' is already loaded....");
    CNR_RETURN_TRUE(logger_, "HW: " + getHwName() + ", CTRL: " + ctrl_to_load_name);
  }

  if (!cm_->loadController(ctrl_to_load_name))
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

  controllers_.emplace( ctrl_to_load_name, cm_->getControllerByName(ctrl_to_load_name) );
  
  CNR_RETURN_TRUE(logger_, "HW: " + getHwName() + ", CTRL: " + ctrl_to_load_name);
}

bool ControllerManagerInterface::switchController(const std::vector<std::string>&  to_start_names,
                                         const std::vector<std::string>&  to_stop_names,
                                         const int                        strictness, 
                                         const ros::Duration&             watchdog)
{
  CNR_TRACE_START(logger_, "HW: "+ getHwName());

  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;

  // ===========================================
  // check of inputs
  if (to_start_names.size())
  {
    CNR_DEBUG(logger_, to_string(to_start_names, "to_start_names  "));
    for (auto const & ctrl : to_start_names)
    {
      if(controllers_.find(ctrl) == controllers_.end() )
      {
        if(!loadController(ctrl, watchdog))
        {
          CNR_ERROR(logger_, "HW: " << getHwName() << "switchController: the controller '" << ctrl
                                <<"' was not loaded, and errors raisen during loading tentative");
          CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
        }
      }
      start_controllers.push_back(ctrl);
    }
  }
  if (to_stop_names.size())
  {
    CNR_DEBUG(logger_, to_string(to_stop_names, "to_stop_names "));
    for (auto const & ctrl : to_stop_names)
    {
      if(controllers_.find(ctrl) == controllers_.end() )
      {
        CNR_ERROR(logger_, "HW: " << getHwName() << "switchController: weird.. you are trying to stop the controller '"
                            << ctrl <<"' that is not in the list of the loaded ctrls.. keep crossed your fingers...");
      }
      else
      {
        stop_controllers.push_back(ctrl);
      }
    }
  }
  // ===========================================

  
  // ===========================================
  // switch controllers
  //
  if ((start_controllers.size() == 0) && (stop_controllers.size() == 0))
  {
    CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
  }

  CNR_DEBUG(logger_, "HW: " + getHwName() + " Call the switchController of the ControllerManager");
  if (!cm_->switchController(start_controllers, stop_controllers, strictness) )
  {
    error_ = "The ControllerManagerInterface failed in switchin the controller. Abort.";
    CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
  }
  CNR_DEBUG(logger_, "HW: " + getHwName() + " Call the switchController of the ControllerManager DONE!!!!!!!");
  // ===========================================

  // =====================================================
  // check if properly switched
  ros::Time st = ros::Time::now();
  while(ros::ok())
  {
    error_ = "";
    for (const std::string& ctrl_name : start_controllers)
    {
      if(!cm_->getControllerByName(ctrl_name))
      {
        error_ += "The controller " + getHwName()+"/" + ctrl_name + " does not still exist...";
      }
      
      if(cm_->getControllerByName(ctrl_name)->state_ != controller_interface::ControllerBase::ControllerState::RUNNING)
      {
        error_ += "The controller " + getHwName()+"/" + ctrl_name + " is in '"
               + ControllerManagerInterface::controllerStateToString(cm_->getControllerByName(ctrl_name)->state_)
               +"' while 'RUNNING' was expected";
      }
    }
    for (const std::string& ctrl_name : stop_controllers)
    {
      if(!cm_->getControllerByName(ctrl_name))
      {
        error_ += "The controller " + getHwName()+"/" + ctrl_name + " does not still exist...";
      }
      if(cm_->getControllerByName(ctrl_name)->state_ != controller_interface::ControllerBase::ControllerState::STOPPED)
      {
        error_ += "The controller " + getHwName()+"/" + ctrl_name + " is in '"
               + ControllerManagerInterface::controllerStateToString(cm_->getControllerByName(ctrl_name)->state_)
               +"' while 'STOPPED' was expected";
      }
    }

    // return false if any error present, and the watchdog expires
    if((error_.length() >0) && ((ros::Time::now() - st).toSec() > watchdog.toSec()) )
    {
      CNR_ERROR(logger_, "HW: " + getHwName()+": " + error_);
      CNR_RETURN_FALSE(logger_);
    }
    else
    {
      CNR_INFO(logger_, "HW: " + getHwName() + " switchController SUCCESS!");    
      break;
    }
  }
  // ======================================================
  CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
}

bool ControllerManagerInterface::listControllers(std::vector<controller_manager_msgs::ControllerState>&  running,
                                        std::vector<controller_manager_msgs::ControllerState>&  stopped,
                                        const ros::Duration&  watchdog)
{
  bool ret = true;
  CNR_TRACE_START(logger_, "HW: "+ getHwName());
  running.clear();
  stopped.clear();
  if(controllers_.size()>0)
  {
    ret = ControllerManagerInterfaceBase::listControllers(running, stopped, watchdog);
  }
  CNR_RETURN_BOOL(logger_, ret, "HW: "+ getHwName());
}

std::vector<std::string>  ControllerManagerInterface::getControllerNames() const
{
  std::vector<std::string> ret;
  for(const auto & ctrl : controllers_)
    ret.push_back(ctrl.first);

  return ret;
}
  

/**
 * @brief ControllerManagerInterface::unloadController
 * @param ctrl_to_unload_name
 * @param watchdog
 * @return
 */
bool ControllerManagerInterface::unloadController(const std::string& ctrl_to_unload_name, const ros::Duration& watchdog)
{
  bool ret = true;
  CNR_TRACE_START(logger_, "HW: "+ getHwName()+", CTRL: " + ctrl_to_unload_name);

  if (!cm_->unloadController( ctrl_to_unload_name ))
  {
    error_ = "The ControllerManagerInterface failed in unloading the controller '"+ctrl_to_unload_name+ "'. Abort.";
    ret = false;
    //CNR_RETURN_FALSE(logger_, "HW: "+ getHwName()+", CTRL: " + ctrl_to_unload_name);
  }

  std::string st = (ret  ? "UNLOADED" : "ERROR_UNLOAD");
  std::vector<std::string> status_history;
  //ros::param::get(cnr::control::ctrl_status_param_name(getHwName(), ctrl_to_unload_name),  status_history);

  status_history.push_back(st);
  //ros::param::set(cnr::control::ctrl_status_param_name(getHwName(), ctrl_to_unload_name),  status_history);
  //ros::param::set(cnr::control::ctrl_last_status_param_name(getHwName(), ctrl_to_unload_name), st) ;

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
 * @brief ControllerManagerInterface::diagnostics
 * @param stat
 */
void ControllerManagerInterface::diagnosticsInfo(diagnostic_updater::DiagnosticStatusWrapper &stat)
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
 * @brief ControllerManagerInterface::diagnostics
 * @param stat
 */
void ControllerManagerInterface::diagnosticsWarn(diagnostic_updater::DiagnosticStatusWrapper &stat)
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
 * @brief ControllerManagerInterface::diagnostics
 * @param stat
 */
void ControllerManagerInterface::diagnosticsError(diagnostic_updater::DiagnosticStatusWrapper &stat)
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

void ControllerManagerInterface::diagnosticsPerformance(diagnostic_updater::DiagnosticStatusWrapper &stat)
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
