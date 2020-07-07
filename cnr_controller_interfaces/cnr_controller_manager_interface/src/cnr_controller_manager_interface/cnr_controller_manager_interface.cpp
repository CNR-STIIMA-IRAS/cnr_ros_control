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





bool ControllerManagerInterface::list_ctrl(std::vector< controller_manager_msgs::ControllerState >&  running
    , std::vector< controller_manager_msgs::ControllerState >&  stopped
    , const ros::Duration&                                      watchdog)
{

  CNR_TRACE_START_THROTTLE(*logger_, 10.0);
  controller_manager_msgs::ListControllers list_ctrls_srv;
  if (!listRequest(list_ctrls_srv, error_, watchdog))
  {
    CNR_RETURN_FALSE(*logger_, error_);
  }

  running.clear();
  stopped.clear();
  for (controller_manager_msgs::ControllerState& ctrl : list_ctrls_srv.response.controller)
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


bool ControllerManagerInterface::load_ctrl(const std::string& ctrl_to_load_name, const ros::Duration&       watchdog)
{
  controller_manager_msgs::LoadController   load_ctrl_srv;
  load_ctrl_srv.request.name = ctrl_to_load_name;
  if (!loadRequest(load_ctrl_srv, error_, watchdog))
  {
    return false;
  }

  if (!load_ctrl_srv.response.ok)
  {
    error_ = "The service '" + loadServiceName() + "' failed while loading '" + ctrl_to_load_name + "'\n";
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


bool ControllerManagerInterface::load_ctrl(const std::vector<std::string>& ctrl_to_load_names, const ros::Duration& watchdog)
{
  for (const std::string& ctrl : ctrl_to_load_names)
  {
    if (!load_ctrl(ctrl, watchdog))
      return false;
  }
  return true;
}


bool ControllerManagerInterface::doswitch(const int                        strictness
    , const std::vector<std::string>*  ctrl_to_load_and_start_names
    , const std::vector<std::string>*  ctrl_to_restart_names
    , const std::vector<std::string>*  ctrl_to_stop_unload_names
    , const ros::Duration&             watchdog)
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
  //if( ctrl_to_restart_names         ) { for( auto const & ctrl : *ctrl_to_restart_names         )   switch_ctrl_srv.request.stop_controllers .push_back(ctrl); }
  if (ctrl_to_stop_unload_names)
  {
    CNR_DEBUG(*logger_, to_string(*ctrl_to_stop_unload_names, "ctrl_to_stop_unload_names "));
    for (auto const & ctrl : *ctrl_to_stop_unload_names)
      switch_ctrl_srv.request.stop_controllers .push_back(ctrl);
  }

  // CNR_DEBUG(*logger_, " Controller Request:\n" << switch_ctrl_srv.request );

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
    error_ = "The service '" + doswitchServiceName() + "' failed. Abort.";
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


bool ControllerManagerInterface::doswitch(const int&                                                    strictness
    , const std::vector<controller_manager_msgs::ControllerState>*  ctrl_to_load_and_start_names
    , const std::vector<controller_manager_msgs::ControllerState>*  ctrl_to_restart_names
    , const std::vector<controller_manager_msgs::ControllerState>*  ctrl_to_stop_unload_names
    , const ros::Duration&                                          watchdog)
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

  bool ret = doswitch(strictness, _ctrl_to_load_and_start_names, _ctrl_to_restart_names, _ctrl_to_stop_unload_names, watchdog);
  CNR_RETURN_BOOL(*logger_, ret);
}


bool ControllerManagerInterface::doswitch(const int strictness, const std::vector<std::string> * const next_ctrl, const ros::Duration& watchdog)
{

  std::vector< controller_manager_msgs::ControllerState > ctrl_running;
  std::vector< controller_manager_msgs::ControllerState > ctrl_stopped;

  if (!list_ctrl(ctrl_running, ctrl_stopped))
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

  if (!load_ctrl(ctrl_to_load_and_start_names, watchdog))
  {
    CNR_RETURN_FALSE(*logger_, "Loading the controllers for HW '" + getHwName() + "' failed. Error: " + error());
  }

  if (!doswitch(strictness, &ctrl_to_load_and_start_names, &ctrl_to_restart_names, &ctrl_to_stop_unload_names, watchdog))
  {
    CNR_RETURN_FALSE(*logger_, "Switching the controllers '" + getHwName() + "'failed. Error: " + error());
  }

  if (!unload_ctrl(ctrl_to_unload_names, watchdog))
  {
    CNR_RETURN_FALSE(*logger_, "Unload the controllers '" + getHwName() + "'failed. Error: " + error());
  }

  if (!match_ctrl(ctrl_next_names, watchdog))
  {
    CNR_RETURN_FALSE(*logger_, "Matching the controllers '" + getHwName() + "'failed. Error: " + error());
  }
  CNR_RETURN_TRUE(*logger_);
}


bool ControllerManagerInterface::unload_ctrl(const std::string& ctrl_to_unload_name, const ros::Duration& watchdog)
{
  bool ret = true;
  CNR_TRACE_START(*logger_);
  controller_manager_msgs::UnloadController  unload_ctrl_srv;
  unload_ctrl_srv.request.name = ctrl_to_unload_name;
  if (!unloadRequest(unload_ctrl_srv, error_, watchdog))
  {
    ret = false;
    CNR_RETURN_FALSE(*logger_);
  }
  else if (!unload_ctrl_srv.response.ok)
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


bool ControllerManagerInterface::unload_ctrl(const std::vector<std::string>& ctrl_to_unload_names, const ros::Duration& watchdog)
{
  CNR_TRACE_START(*logger_);
  std::vector< controller_manager_msgs::ControllerState >  running;
  std::vector< controller_manager_msgs::ControllerState >  stopped;
  if (!list_ctrl(running, stopped, watchdog))
  {
    CNR_RETURN_FALSE(*logger_, error());
  }
  std::vector< std::string > rr = cnr_controller_interface::get_names(running);
  std::vector< std::string > ss = cnr_controller_interface::get_names(stopped);
  CNR_DEBUG(*logger_, " 1# Running: " << ::cnr_controller_manager_interface::to_string(rr));
  CNR_DEBUG(*logger_, " 1# Stopped: " << ::cnr_controller_manager_interface::to_string(ss));

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
      if (!stop_unload_ctrl(to_be_stopped, watchdog))
      {
        CNR_RETURN_FALSE(*logger_);
      }

      rr = cnr_controller_interface::get_names(running);
      ss = cnr_controller_interface::get_names(stopped);
      CNR_DEBUG(*logger_, " 2# Running: " << ::cnr_controller_manager_interface::to_string(rr));
      CNR_DEBUG(*logger_, " 2# Stopped: " << ::cnr_controller_manager_interface::to_string(ss));
    }
  }

  for (const auto s : ss)
  {
    if (std::find(ctrl_to_unload_names.begin(), ctrl_to_unload_names.end(), s) != ctrl_to_unload_names.end())
    {
      if (!unload_ctrl(s, watchdog))
      {
        CNR_RETURN_FALSE(*logger_);
      }
    }
  }

  CNR_RETURN_TRUE(*logger_);
}


bool ControllerManagerInterface::unload_ctrl(const std::vector<controller_manager_msgs::ControllerState>& ctrl_to_unload_names, const ros::Duration& watchdog)
{
  CNR_TRACE_START(*logger_);
  std::vector<std::string> _ctrl_to_unload_names(ctrl_to_unload_names.size()) ;
  std::transform(ctrl_to_unload_names.begin(), ctrl_to_unload_names.end(), _ctrl_to_unload_names.begin(), [](auto v)
  {
    return v.name;
  });

  bool ret = unload_ctrl(_ctrl_to_unload_names, watchdog);
  CNR_RETURN_BOOL(*logger_, ret);
}


bool ControllerManagerInterface::match_ctrl(const std::vector<std::string>& ctrl_names, const ros::Duration&            watchdog)
{
  std::vector<controller_manager_msgs::ControllerState> running;
  std::vector<controller_manager_msgs::ControllerState> stopped;

  if (!list_ctrl(running, stopped, watchdog))
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

bool ControllerManagerInterface::stop_unload_ctrl(const std::vector<std::string>&  ctrl_to_stop_unload_names, const ros::Duration&  watchdog)
{
  CNR_TRACE_START(*logger_);

  if (!doswitch(1, nullptr, nullptr, &ctrl_to_stop_unload_names, watchdog))
  {
    CNR_RETURN_FALSE(*logger_);
  }

  bool ret = unload_ctrl(ctrl_to_stop_unload_names, watchdog);
  CNR_RETURN_BOOL(*logger_, ret);
}


bool ControllerManagerInterface::stop_unload_all_ctrl(const ros::Duration&  watchdog)
{

  CNR_TRACE_START(*logger_);
  std::vector< controller_manager_msgs::ControllerState >  running;
  std::vector< controller_manager_msgs::ControllerState >  stopped;

  if (!list_ctrl(running, stopped, watchdog))
  {
    CNR_RETURN_FALSE(*logger_);
  }

  std::vector<std::string> ctrls_running = cnr_controller_interface::get_names(running);
  std::vector<std::string> ctrls_stopped = cnr_controller_interface::get_names(stopped);
  std::vector<std::string> ctrls = ctrls_running;
  ctrls.insert(ctrls.end(), ctrls_stopped.begin(), ctrls_stopped.end());

  CNR_DEBUG(*logger_, "Try to stop " << to_string(ctrls));
  bool ret = stop_unload_ctrl(ctrls, watchdog);
  CNR_RETURN_BOOL(*logger_, ret);

}

bool stop_unload_all_ctrl(std::map<std::string, cnr_controller_manager_interface::ControllerManagerInterface>& ctrl_srvs
                          , const std::vector<std::string>& hw_to_unload_names
                          , const ros::Duration&            watchdog)
{

  if (ctrl_srvs.size() > 0)
    CNR_TRACE_START(* ctrl_srvs.begin()->second.getLogger());

  std::map<std::string, std::thread* > stoppers;
  std::map< std::string, bool > unload_ok;

  for (auto const & hw_name : hw_to_unload_names)
  {
    unload_ok[hw_name] = false;
  }
  auto stopper = [ & ](const std::string & hw_name, cnr_controller_manager_interface::ControllerManagerInterface & ctrl, bool & ok)
  {
    CNR_INFO(*ctrl.getLogger(), "Try stopping the controllers of HW '" + hw_name + "'");
    ok = ctrl.stop_unload_all_ctrl(watchdog);
    if (ok)
    {
      CNR_INFO(*ctrl.getLogger(), "Successful stopping the controllers of HW '" + hw_name + "'");
    }
    else
    {
      CNR_ERROR(*ctrl.getLogger(), "Error in stopping the controllers of HW '" + hw_name + "': " + ctrl.error());
    }
  };
  for (auto const & hw_name : hw_to_unload_names)
  {
    stoppers[hw_name] = new std::thread(stopper, hw_name,  std::ref(ctrl_srvs.at(hw_name)), std::ref(unload_ok[hw_name]));
  }
  for (auto& thread : stoppers)
  {
    CNR_DEBUG(*(ctrl_srvs.at(thread.first).getLogger()), thread.first << ": Waiting for thread join execution ...");
    if (thread.second->joinable())
    {
      thread.second->join();
    }
    CNR_DEBUG(*ctrl_srvs.at(thread.first).getLogger(), thread.first << ": Thread Finished ");
  }

  bool ok = true;
  std::for_each(unload_ok.begin(), unload_ok.end(), [&](std::pair<std::string, bool> b)
  {
    ok &= b.second;
  });

  if (ctrl_srvs.size() > 0)
    CNR_RETURN_BOOL(* ctrl_srvs.begin()->second.getLogger(), ok);

  return ok;
}


}

