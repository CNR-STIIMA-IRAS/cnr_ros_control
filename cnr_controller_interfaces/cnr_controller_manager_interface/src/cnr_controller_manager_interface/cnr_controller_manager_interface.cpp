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

#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_controller_manager_interface/cnr_controller_manager_interface.h>

namespace cnr_controller_manager_interface
{


/**
 * 
 * 
 * 
 * 
 * 
 */
ControllerManagerBase::ControllerManagerBase(std::shared_ptr<cnr_logger::TraceLogger> log,
                                             const std::string& hw_name)
: nh_("/" + hw_name), logger_(log)
{
  list_       = nh_.serviceClient<controller_manager_msgs::ListControllers> ("/" + hw_name + "/controller_manager/list_controllers");
  list_types_ = nh_.serviceClient<controller_manager_msgs::ListControllerTypes> ("/" + hw_name + "/controller_manager/list_controller_types");
  assert(logger_);
}

ControllerManagerBase::~ControllerManagerBase()
{
  stopUnloadAllControllers();
}

bool ControllerManagerBase::listRequest(controller_manager_msgs::ListControllers& msg,
                                             std::string& error,
                                             const ros::Duration& watchdog)
{
  return callRequest(list_, msg, error, watchdog);
}
bool ControllerManagerBase:: listTypeRequest(controller_manager_msgs::ListControllerTypes& msg,
                                                  std::string& error,
                                                  const ros::Duration& watchdog)
{
  return callRequest(list_types_, msg, error, watchdog);
}

bool ControllerManagerBase::listControllers(std::vector< controller_manager_msgs::ControllerState >&  running,
    std::vector< controller_manager_msgs::ControllerState >&  stopped,
    const ros::Duration&                                      watchdog)
{

  CNR_TRACE_START_THROTTLE(*logger_, 10.0);
  controller_manager_msgs::ListControllers listControllerss_srv;
  if (!callRequest(list_, listControllerss_srv, error_, watchdog))
  {
    CNR_RETURN_FALSE(*logger_, error_);
  }

  running.clear();
  stopped.clear();
  for (controller_manager_msgs::ControllerState& ctrl : listControllerss_srv.response.controller)
  {
    if (ctrl.state == "running")
    {
      running.push_back(ctrl);
    }
    else
    {
      stopped.push_back(ctrl);
    }
  }
  CNR_RETURN_TRUE_THROTTLE(*logger_, 10.0);
}

bool ControllerManagerBase::matchControllers(const std::vector<std::string>& ctrl_names, const ros::Duration& watchdog)
{
  std::vector<controller_manager_msgs::ControllerState> running;
  std::vector<controller_manager_msgs::ControllerState> stopped;

  if (!listControllers(running, stopped, watchdog))
  {
    error_ = "Failed in getting the controllers info: " + error_;
    return false;
  }
  std::vector<std::string> running_names = cnr_controller_interface::get_names(running);
  std::vector<std::string> stopped_names = cnr_controller_interface::get_names(stopped);
  running_names.insert(running_names.end(), stopped_names.begin(), stopped_names.end());

  if (ctrl_names.size() != running_names.size())
  {
    return false;
  }
  for (auto const l : ctrl_names)
  {
    if (std::find(running_names.begin(), running_names.end(), l) == running_names.end()) return false;
  }
  return true;
}

bool ControllerManagerBase::loadControllers(const std::vector<std::string>& ctrl_to_load_names, const ros::Duration& watchdog)
{
  for (const std::string& ctrl : ctrl_to_load_names)
  {
    if (!loadController(ctrl, watchdog))
      return false;
  }
  return true;
}


bool ControllerManagerBase::switchControllers(const int&                                                    strictness,
                                              const std::vector<controller_manager_msgs::ControllerState>*  ctrl_to_load_and_start_names,
                                              const std::vector<controller_manager_msgs::ControllerState>*  ctrl_to_restart_names,
                                              const std::vector<controller_manager_msgs::ControllerState>*  ctrl_to_stop_unload_names,
                                              const ros::Duration&                                          watchdog)
{
  CNR_TRACE_START(*logger_);
  std::vector<std::string>*  _ctrl_to_load_and_start_names  = ctrl_to_load_and_start_names  ? new std::vector<std::string>(ctrl_to_load_and_start_names ->size()) : nullptr;
  std::vector<std::string>*  _ctrl_to_restart_names         = ctrl_to_restart_names         ? new std::vector<std::string>(ctrl_to_restart_names        ->size()) : nullptr;
  std::vector<std::string>*  _ctrl_to_stop_unload_names = ctrl_to_stop_unload_names ? new std::vector<std::string>(ctrl_to_stop_unload_names->size()) : nullptr;

  if (_ctrl_to_load_and_start_names) std::transform(ctrl_to_load_and_start_names  ->begin(), ctrl_to_load_and_start_names  ->end(), _ctrl_to_load_and_start_names  ->begin(), [](auto v)
  {
    return v.name;
  });
  if (_ctrl_to_restart_names) std::transform(ctrl_to_restart_names         ->begin(), ctrl_to_restart_names         ->end(), _ctrl_to_restart_names         ->begin(), [](auto v)
  {
    return v.name;
  });
  if (_ctrl_to_stop_unload_names) std::transform(ctrl_to_stop_unload_names ->begin(), ctrl_to_stop_unload_names ->end(), _ctrl_to_stop_unload_names ->begin(), [](auto v)
  {
    return v.name;
  });

  bool ret = switchController(strictness, _ctrl_to_load_and_start_names, _ctrl_to_restart_names, _ctrl_to_stop_unload_names, watchdog);
  CNR_RETURN_BOOL(*logger_, ret);
}

bool ControllerManagerBase::switchControllers(const int strictness,
                                              const std::vector<std::string> *
                                              const next_ctrl,
                                              const ros::Duration& watchdog)
{

  std::vector< controller_manager_msgs::ControllerState > ctrl_running;
  std::vector< controller_manager_msgs::ControllerState > ctrl_stopped;

  if (!listControllers(ctrl_running, ctrl_stopped))
  {
    CNR_RETURN_FALSE(*logger_, "Getting the controller info failed: " + error_);
  }

  std::vector<std::string> ctrl_running_names = cnr_controller_interface::get_names(ctrl_running);
  std::vector<std::string> ctrl_stopped_names = cnr_controller_interface::get_names(ctrl_stopped);
  std::vector<std::string> ctrl_loaded_names  = ctrl_running_names;
  ctrl_loaded_names.insert(ctrl_loaded_names.end(), ctrl_stopped_names.begin(), ctrl_stopped_names.end());
  std::vector<std::string> ctrl_to_load_and_start_names;

  std::vector<std::string> ctrl_to_stop_and_restart_names;
  std::vector<std::string> ctrl_to_stop_unload_names;
  std::vector<std::string> ctrl_to_restart_names;
  std::vector<std::string> ctrl_to_unload_names;

  std::vector<std::string> ctrl_next_names = next_ctrl ? *next_ctrl : std::vector<std::string>();
  /*
  void extract( const std::vector< T >& va
            , const std::vector< T >& vb
            , std::vector< T >* a_not_in_b = nullptr
            , std::vector< T >* b_not_in_a = nullptr
            , std::vector< T >* a_in_b     = nullptr)*/
  extract(ctrl_next_names, ctrl_running_names, &ctrl_to_load_and_start_names, &ctrl_to_stop_unload_names, &ctrl_to_stop_and_restart_names);
  //--
  CNR_DEBUG(*logger_, to_string(ctrl_next_names, "ctrl_next_names               "));
  CNR_DEBUG(*logger_, to_string(ctrl_running_names, "ctrl_running_names            "));
  CNR_DEBUG(*logger_, to_string(ctrl_stopped_names, "ctrl_stopped_names            "));
  CNR_DEBUG(*logger_, to_string(ctrl_loaded_names, "ctrl_loaded_names             "));
  CNR_DEBUG(*logger_, to_string(ctrl_to_stop_unload_names, "ctrl_to_stop_unload_names "));
  CNR_DEBUG(*logger_, to_string(ctrl_to_stop_and_restart_names, "ctrl_to_stop_and_restart_names"));

  std::vector<std::string> ctrl_to_load_and_start_names_;
  extract(ctrl_to_load_and_start_names, ctrl_stopped_names, &ctrl_to_load_and_start_names_, &ctrl_to_unload_names, &ctrl_to_restart_names);
  ctrl_to_load_and_start_names = ctrl_to_load_and_start_names_;

  ctrl_to_restart_names.insert(ctrl_to_restart_names.begin(), ctrl_to_stop_and_restart_names.begin(), ctrl_to_stop_and_restart_names.end());
  ctrl_to_unload_names .insert(ctrl_to_unload_names.begin(),  ctrl_to_stop_unload_names.begin(),  ctrl_to_stop_unload_names.end());

  //--
  CNR_DEBUG(*logger_, to_string(ctrl_to_load_and_start_names, "ctrl_to_load_and_start_names  "));
  CNR_DEBUG(*logger_, to_string(ctrl_to_unload_names, "ctrl_to_unload_names          "));
  CNR_DEBUG(*logger_, to_string(ctrl_to_restart_names, "ctrl_to_restart_names         "));

  if (!loadControllers(ctrl_to_load_and_start_names, watchdog))
  {
    CNR_RETURN_FALSE(*logger_, "Loading the controllers for HW '" + getHwName() + "' failed. Error: " + error());
  }

  if (!switchController(strictness, &ctrl_to_load_and_start_names, &ctrl_to_restart_names, &ctrl_to_stop_unload_names, watchdog))
  {
    CNR_RETURN_FALSE(*logger_, "Switching the controllers '" + getHwName() + "'failed. Error: " + error());
  }

  if (!unloadControllers(ctrl_to_unload_names, watchdog))
  {
    CNR_RETURN_FALSE(*logger_, "Unload the controllers '" + getHwName() + "'failed. Error: " + error());
  }

  if (!matchControllers(ctrl_next_names, watchdog))
  {
    CNR_RETURN_FALSE(*logger_, "Matching the controllers '" + getHwName() + "'failed. Error: " + error());
  }
  CNR_RETURN_TRUE(*logger_);
}

bool ControllerManagerBase::unloadControllers(const std::vector<std::string>& ctrl_to_unload_names,
                                                   const ros::Duration& watchdog)
{
  CNR_TRACE_START(*logger_);
  std::vector< controller_manager_msgs::ControllerState >  running;
  std::vector< controller_manager_msgs::ControllerState >  stopped;
  if (!listControllers(running, stopped, watchdog))
  {
    CNR_RETURN_FALSE(*logger_, error());
  }

  std::vector< std::string > rr = cnr_controller_interface::get_names(running);
  std::vector< std::string > ss = cnr_controller_interface::get_names(stopped);
  if (rr.size() > 0)
  {
    std::vector<std::string> to_be_stopped;
    for (const auto r : rr)
    {
      if (std::find(ctrl_to_unload_names.begin(), ctrl_to_unload_names.end(), r) != ctrl_to_unload_names.end())
      {
        to_be_stopped.push_back(r);
      }
    }

    if (to_be_stopped.size() > 0)
    {
      CNR_ERROR(*logger_, "It is not possibile to unloade controller, since there still running controllers");
      CNR_RETURN_FALSE(*logger_);
    }
  }

  for (const auto s : ss)
  {
    if (std::find(ctrl_to_unload_names.begin(), ctrl_to_unload_names.end(), s) != ctrl_to_unload_names.end())
    {
      if (!unloadController(s, watchdog))
      {
        CNR_RETURN_FALSE(*logger_);
      }
    }
  }

  CNR_RETURN_TRUE(*logger_);
}

bool ControllerManagerBase::unloadControllers(const std::vector<controller_manager_msgs::ControllerState>& ctrl_to_unload_names,
                                                   const ros::Duration& watchdog)
{
  CNR_TRACE_START(*logger_);
  std::vector<std::string> _ctrl_to_unload_names(ctrl_to_unload_names.size()) ;
  std::transform(ctrl_to_unload_names.begin(), ctrl_to_unload_names.end(), _ctrl_to_unload_names.begin(), [](auto v)
  {
    return v.name;
  });

  bool ret = unloadControllers(_ctrl_to_unload_names, watchdog);
  CNR_RETURN_BOOL(*logger_, ret);
}

bool ControllerManagerBase::stopUnloadControllers(const std::vector<std::string>&  ctrl_to_stop_unload_names,
                                                       const ros::Duration&  watchdog)
{
  CNR_TRACE_START(*logger_);

  if (!switchController(1, nullptr, nullptr, &ctrl_to_stop_unload_names, watchdog))
  {
    CNR_RETURN_FALSE(*logger_);
  }

  bool ret = unloadControllers(ctrl_to_stop_unload_names, watchdog);
  CNR_RETURN_BOOL(*logger_, ret);
}

bool ControllerManagerBase::stopUnloadAllControllers(const ros::Duration&  watchdog)
{
  CNR_TRACE_START(*logger_);
  std::vector< controller_manager_msgs::ControllerState >  stopped;

  do
  {
    std::vector< controller_manager_msgs::ControllerState >  running;

    if (!listControllers(running, stopped, watchdog))
    {
      CNR_RETURN_FALSE(*logger_);
    }

    if( running.size() > 0 )
    {
      std::vector<std::string> ctrls_running = cnr_controller_interface::get_names(running);
      CNR_DEBUG(*logger_, "Try to stop " << to_string(ctrls_running));
      if(!switchController(1, nullptr, nullptr, &ctrls_running, watchdog) )
      {
        CNR_RETURN_FALSE(*logger_, "Error in stopping controllers");
      }
    }
    else
    {
      break;
    }
  } while( ros::ok() );

  std::vector<std::string> ctrls_stopped = cnr_controller_interface::get_names(stopped);

  CNR_DEBUG(*logger_, "Try to stop " << to_string(ctrls_stopped));
  bool ret = unloadControllers(ctrls_stopped, watchdog);
  CNR_RETURN_BOOL(*logger_, ret);
}



/**
 * 
 * 
 * 
 * 
 * 
 */
ControllerManagerInterface::ControllerManagerInterface(std::shared_ptr<cnr_logger::TraceLogger> log,
                                                                   const std::string& hw_name)
: ControllerManagerBase( log, hw_name )
{
  load_     = nh_.serviceClient<controller_manager_msgs::LoadController> ("/" + hw_name + "/controller_manager/load_controller");
  unload_   = nh_.serviceClient<controller_manager_msgs::UnloadController> ("/" + hw_name + "/controller_manager/unload_controller");
  doswitch_ = nh_.serviceClient<controller_manager_msgs::SwitchController> ("/" + hw_name + "/controller_manager/switch_controller");
}

bool ControllerManagerInterface::loadRequest(controller_manager_msgs::LoadController& msg,
                                                   std::string& error,
                                                   const ros::Duration& watchdog)
{
  return callRequest(load_, msg, error, watchdog);
}
bool ControllerManagerInterface::unloadRequest(controller_manager_msgs::UnloadController& msg,
                                                     std::string& error,
                                                     const ros::Duration& watchdog)
{
  return callRequest(unload_, msg, error, watchdog);
}
bool ControllerManagerInterface::switchRequest(controller_manager_msgs::SwitchController& msg,
                                                     std::string& error,
                                                     const ros::Duration& watchdog)
{
  return callRequest(doswitch_, msg, error, watchdog);
}

bool ControllerManagerInterface::loadController(const std::string& to_load_name, const ros::Duration& watchdog)
{
  controller_manager_msgs::LoadController   loadController_srv;
  loadController_srv.request.name = to_load_name;
  if (!loadRequest(loadController_srv, error_, watchdog))
  {
    return false;
  }

  if (!loadController_srv.response.ok)
  {
    error_ = "The service '" + loadServiceName() + "' failed while loading '" + to_load_name + "'\n";
    controller_manager_msgs::ListControllerTypes ctrl_types;
    if (!listTypeRequest(ctrl_types, error_, watchdog))
    {
      return false;
    }
    error_ += "Available " + std::to_string((int)(ctrl_types.response.types.size())) +  "# classes: ";
    for (auto const & t : ctrl_types.response.types)
    {
      error_ += "-" + t + "\n" ;
    }
    return false;
  }

  const std::string n = cnr_controller_interface::ctrl_list_param(getHwName());
  std::vector<std::string> l;
  if (ros::param::has(n))
  {
    ros::param::get(n, l);
  }
  if (std::find(l.begin(), l.end(), to_load_name) == l.end())
  {
    l.push_back(to_load_name);
    ros::param::set(n, l);
  }
  return true;
}

bool ControllerManagerInterface::switchController(const int                        strictness                  ,
                                                  const std::vector<std::string>*  ctrl_to_load_and_start_names,
                                                  const std::vector<std::string>*  ctrl_to_restart_names       ,
                                                  const std::vector<std::string>*  ctrl_to_stop_unload_names   ,
                                                  const ros::Duration&             watchdog                    )
{

  CNR_TRACE_START(*logger_);

  controller_manager_msgs::SwitchController switch_ctrl_srv;
  switch_ctrl_srv.request.strictness = (strictness == 0) ? 1 : strictness;
  switch_ctrl_srv.request.start_controllers.resize(0);
  switch_ctrl_srv.request.stop_controllers.resize(0);

  //--
  if (ctrl_to_load_and_start_names)
  {
    CNR_DEBUG(*logger_, to_string(*ctrl_to_load_and_start_names, "ctrl_to_load_and_start_names  "));
    for (auto const & ctrl : *ctrl_to_load_and_start_names)
      switch_ctrl_srv.request.start_controllers.push_back(ctrl);
  }
  if (ctrl_to_restart_names)
  {
    CNR_DEBUG(*logger_, to_string(*ctrl_to_restart_names, "ctrl_to_restart_names         "));
    for (auto const & ctrl : *ctrl_to_restart_names)
      switch_ctrl_srv.request.start_controllers.push_back(ctrl);
  }
  if (ctrl_to_stop_unload_names)
  {
    CNR_DEBUG(*logger_, to_string(*ctrl_to_stop_unload_names, "ctrl_to_stop_unload_names "));
    for (auto const & ctrl : *ctrl_to_stop_unload_names)
      switch_ctrl_srv.request.stop_controllers .push_back(ctrl);
  }


  // switch controllers
  if (switch_ctrl_srv.request.start_controllers.size() == 0 && switch_ctrl_srv.request.stop_controllers.size() == 0)
  {
    CNR_RETURN_TRUE(*logger_);
  }

  if (!switchRequest(switch_ctrl_srv, error_, watchdog))
  {
    CNR_RETURN_FALSE(*logger_);
  }
  if (!switch_ctrl_srv.response.ok)
  {
    error_ = "The service '" + switchControllerServiceName() + "' failed. Abort.";
    CNR_RETURN_FALSE(*logger_);
  }
  // CNR_DEBUG(*logger_, " Controller Request DONE\n" );

  if (watchdog.toSec() > 0)
  {
    // check the param, and exit only if the state is running!
    for (const std::string ctrl_name : switch_ctrl_srv.request.start_controllers)
    {
      std::string status;
      if (!cnr_controller_interface::get_state(getNamespace(), ctrl_name, status, error_, watchdog))
      {
        CNR_RETURN_FALSE(*logger_);
      }
      if ((status != "INITIALIZED") && (status != "RUNNING"))
      {
        error_ += "Failed while checking '" + ctrl_name + "' state. The state is " + status + " while RUNNING is expected";
        CNR_RETURN_FALSE(*logger_);
      }
    }
    for (const std::string ctrl_name : switch_ctrl_srv.request.stop_controllers)
    {
      std::string status;
      if (!cnr_controller_interface::get_state(getNamespace(), ctrl_name, status, error_, watchdog))
      {
        CNR_RETURN_FALSE(*logger_);
      }
      if (status != "STOPPED")
      {
        error_ += "Failed while checking '" + ctrl_name + "' state. The state is " + status + " while STOPPED is expected";
        CNR_RETURN_FALSE(*logger_);
      }
    }
  }
  CNR_RETURN_TRUE(*logger_);
}


bool ControllerManagerInterface::unloadController(const std::string& ctrl_to_unload_name, const ros::Duration& watchdog)
{
  bool ret = true;
  CNR_TRACE_START(*logger_);
  controller_manager_msgs::UnloadController  unloadController_srv;
  unloadController_srv.request.name = ctrl_to_unload_name;
  if (!unloadRequest(unloadController_srv, error_, watchdog))
  {
    ret = false;
    CNR_RETURN_FALSE(*logger_);
  }
  else if (!unloadController_srv.response.ok)
  {
    error_ = "The service '" + unloadServiceName() + "' failed. Abort.";
    ret = false;
    CNR_RETURN_FALSE(*logger_);
  }

  std::string st = (ret  ? "UNLOADED" : "ERROR_UNLOAD");
  std::vector<std::string> status_history;
  ros::param::get(cnr_controller_interface::status_param(getHwName(), ctrl_to_unload_name),  status_history);

  status_history.push_back(st);
  ros::param::set(cnr_controller_interface::status_param(getHwName(), ctrl_to_unload_name),  status_history);
  ros::param::set(cnr_controller_interface::last_status_param(getHwName(), ctrl_to_unload_name), st) ;

  CNR_RETURN_BOOL(*logger_, ret);
}





/**
 * 
 * 
 * 
 * 
 * 
 */
ControllerManager::ControllerManager(std::shared_ptr<cnr_logger::TraceLogger> log,
                                                             const std::string& hw_name,
                                                             hardware_interface::RobotHW *robot_hw,
                                                             const ros::NodeHandle& nh)
: ControllerManagerBase( log, hw_name ) , cm_( robot_hw, nh )
{
  
}

bool ControllerManager::loadController(const std::string& ctrl_to_load_name, const ros::Duration& watchdog)
{
  if (!cm_.loadController(ctrl_to_load_name))
  {
    error_ = "ControllerManagerfailed while loading '" + ctrl_to_load_name + "'\n";
    controller_manager_msgs::ListControllerTypes ctrl_types;
    if (!listTypeRequest(ctrl_types, error_, watchdog))
    {
      return false;
    }
    error_ += "Available " + std::to_string((int)(ctrl_types.response.types.size())) +  "# classes: ";
    for (auto const & t : ctrl_types.response.types)
    {
      error_ += "-" + t + "\n" ;
    }
    return false;
  }

  const std::string n = cnr_controller_interface::ctrl_list_param(getHwName());
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
  return true;
}



bool ControllerManager::switchController(const int                        strictness,
                                         const std::vector<std::string>*  ctrl_to_load_and_start_names,
                                         const std::vector<std::string>*  ctrl_to_restart_names,
                                         const std::vector<std::string>*  ctrl_to_stop_unload_names,
                                         const ros::Duration&             watchdog)
{
  CNR_TRACE_START(*logger_);

  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;

  //--
  if (ctrl_to_load_and_start_names)
  {
    CNR_DEBUG(*logger_, to_string(*ctrl_to_load_and_start_names, "ctrl_to_load_and_start_names  "));
    for (auto const & ctrl : *ctrl_to_load_and_start_names)
      start_controllers.push_back(ctrl);
  }
  if (ctrl_to_restart_names)
  {
    CNR_DEBUG(*logger_, to_string(*ctrl_to_restart_names, "ctrl_to_restart_names         "));
    for (auto const & ctrl : *ctrl_to_restart_names)
      start_controllers.push_back(ctrl);
  }
  if (ctrl_to_stop_unload_names)
  {
    CNR_DEBUG(*logger_, to_string(*ctrl_to_stop_unload_names, "ctrl_to_stop_unload_names "));
    for (auto const & ctrl : *ctrl_to_stop_unload_names)
      stop_controllers.push_back(ctrl);
  }

  // switch controllers
  if ((start_controllers.size() == 0) && (stop_controllers.size() == 0))
  {
    CNR_RETURN_TRUE(*logger_);
  }

  if (!cm_.switchController(start_controllers, stop_controllers, strictness) )
  {
    error_ = "The ControllerManager failed in switchin the controller. Abort.";
    CNR_RETURN_FALSE(*logger_);
  }

  if (watchdog.toSec() > 0)
  {
    // check the param, and exit only if the state is running!
    for (const std::string ctrl_name : start_controllers)
    {
      std::string status;
      if (!cnr_controller_interface::get_state(getNamespace(), ctrl_name, status, error_, watchdog))
      {
        CNR_RETURN_FALSE(*logger_);
      }
      if ((status != "INITIALIZED") && (status != "RUNNING"))
      {
        error_ += "Failed while checking '" + ctrl_name + "' state. The state is '" + status + "' while RUNNING is expected";
        CNR_RETURN_FALSE(*logger_);
      }
    }
    for (const std::string ctrl_name : stop_controllers)
    {
      std::string status;
      if (!cnr_controller_interface::get_state(getNamespace(), ctrl_name, status, error_, watchdog))
      {
        CNR_RETURN_FALSE(*logger_);
      }
      if (status != "STOPPED")
      {
        error_ += "Failed while checking '" + ctrl_name + "' state. The state is '" + status + "' while STOPPED is expected";
        CNR_RETURN_FALSE(*logger_);
      }
    }
  }
  CNR_RETURN_TRUE(*logger_);
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
  CNR_TRACE_START(*logger_);

  if (!cm_.unloadController( ctrl_to_unload_name ))
  {
    error_ = "The ControllerManager failed in unloading the controller '"+ctrl_to_unload_name+ "'. Abort.";
    ret = false;
    CNR_RETURN_FALSE(*logger_);
  }

  std::string st = (ret  ? "UNLOADED" : "ERROR_UNLOAD");
  std::vector<std::string> status_history;
  ros::param::get(cnr_controller_interface::status_param(getHwName(), ctrl_to_unload_name),  status_history);

  status_history.push_back(st);
  ros::param::set(cnr_controller_interface::status_param(getHwName(), ctrl_to_unload_name),  status_history);
  ros::param::set(cnr_controller_interface::last_status_param(getHwName(), ctrl_to_unload_name), st) ;

  CNR_RETURN_BOOL(*logger_, ret);
}


}

