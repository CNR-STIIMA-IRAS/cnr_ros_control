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
#include <cstring>
#include <atomic>
#include <string>
#include <thread>

#include <cnr_controller_interface_params/cnr_controller_interface_params.h>
#include <cnr_controller_manager_interface/cnr_controller_manager_interface_srv.h>

namespace cnr_controller_manager_interface
{




/**
 * 
 * ControllerManagerInterfaceSrv
 * 
 * 
 * 
 */
ControllerManagerInterfaceSrv::ControllerManagerInterfaceSrv(const cnr_logger::TraceLoggerPtr& logger,
                                                       const std::string&       hw_name,
                                                       const bool&              use_proxy)
: ControllerManagerInterfaceBase( logger, hw_name )
{
  const std::string ns = use_proxy ? ("/" + hw_name + "/controller_manager")
                                   : ("/" + hw_name + "/controller_manager_proxy");

  load_     = nh_.serviceClient<controller_manager_msgs::LoadController>   (ns+"/load_controller");
  unload_   = nh_.serviceClient<controller_manager_msgs::UnloadController> (ns+"/unload_controller");
  doswitch_ = nh_.serviceClient<controller_manager_msgs::SwitchController> (ns+"/switch_controller");
}

ControllerManagerInterfaceSrv::~ControllerManagerInterfaceSrv()
{
  this->stopUnloadAllControllers();
  load_.shutdown();
  unload_.shutdown();
  doswitch_.shutdown();
}

bool ControllerManagerInterfaceSrv::loadRequest(controller_manager_msgs::LoadController& msg,
                                             std::string& error,
                                             const ros::Duration& watchdog)
{
  return callRequest(mtx_, load_, msg, error, watchdog);
}
bool ControllerManagerInterfaceSrv::unloadRequest(controller_manager_msgs::UnloadController& msg,
                                               std::string& error,
                                               const ros::Duration& watchdog)
{
  return callRequest(mtx_, unload_, msg, error, watchdog);
}
bool ControllerManagerInterfaceSrv::switchRequest(controller_manager_msgs::SwitchController& msg,
                                               std::string& error,
                                               const ros::Duration& watchdog)
{
  return callRequest(mtx_, doswitch_, msg, error, watchdog);
}

bool ControllerManagerInterfaceSrv::loadController(const std::string& to_load_name, const ros::Duration& watchdog)
{
  CNR_TRACE_START(logger_, "HW: " + getHwName() + ", CTRL: " + to_load_name);
  controller_manager_msgs::LoadController   loadController_srv;
  loadController_srv.request.name = to_load_name;
  if (!loadRequest(loadController_srv, error_, watchdog))
  {
    CNR_RETURN_FALSE(logger_, "HW: " + getHwName() + ", CTRL: " + to_load_name);
  }

  if (!loadController_srv.response.ok)
  {
    error_ = "The service '" + loadServiceName() + "' failed while loading '" + to_load_name + "'\n";
    controller_manager_msgs::ListControllerTypes ctrl_types;
    if (!listTypeRequest(ctrl_types, error_, watchdog))
    {
      CNR_RETURN_FALSE(logger_, "HW: " + getHwName() + ", CTRL: " + to_load_name);
    }
    error_ += "Available " + std::to_string((int)(ctrl_types.response.types.size())) +  "# classes:\n";
    for (auto const & t : ctrl_types.response.types)
    {
      error_ += "-" + t + "\n" ;
    }
    CNR_RETURN_FALSE(logger_, "HW: " + getHwName() + ", CTRL: " + to_load_name);
  }

  CNR_RETURN_TRUE(logger_, "HW: " + getHwName() + ", CTRL: " + to_load_name);
}

bool ControllerManagerInterfaceSrv::switchController(const std::vector<std::string>&  to_start_names,
                                                  const std::vector<std::string>&  to_stop_names   ,
                                                  const int                        strictness             ,
                                                  const ros::Duration&             watchdog               )
{

  CNR_TRACE_START(logger_, "HW: " + getHwName() );

  controller_manager_msgs::SwitchController switch_ctrl_srv;
  switch_ctrl_srv.request.strictness = (strictness == 0) ? 1 : strictness;
  switch_ctrl_srv.request.start_controllers.resize(0);
  switch_ctrl_srv.request.stop_controllers.resize(0);

  auto check = [] (const std::vector<std::string>& ptr)
  {
      return ptr.size()>0;
  };

  CNR_DEBUG(logger_, "HW: " + getHwName() + " -----------------");
  CNR_DEBUG(logger_, "HW: " + getHwName() + " Stricness? " << strictness);
  CNR_DEBUG(logger_, "HW: " + getHwName() + " Controllers to Load/Start?  " + std::string(check(to_start_names) ? "YES" : "NO"));
  CNR_DEBUG(logger_, "HW: " + getHwName() + " Controllers to Stop/Unload? " + std::string(check(to_stop_names) ? "YES" : "NO"));
  CNR_DEBUG(logger_, "HW: " + getHwName() + " -----------------");
  //--
  if(check(to_start_names))
  {
    CNR_DEBUG(logger_, "HW: " + getHwName() + to_string(to_start_names, "Controllers to load/start...: "));
    for (auto const & ctrl : to_start_names)
      switch_ctrl_srv.request.start_controllers.push_back(ctrl);
  }

//  if((strictness!=1) && check(to_restart_names))
//  {
//    CNR_DEBUG(logger_, "HW: " + getHwName() + to_string(to_restart_names, " Controllers to Restart......: "));
//    for (auto const & ctrl : to_restart_names)
//    {
//      switch_ctrl_srv.request.start_controllers.push_back(ctrl);
//    }
//  }

  if(check(to_stop_names))
  {
    CNR_DEBUG(logger_, "HW: " + getHwName() + to_string(to_stop_names, " Controllers to stop/unload..: "));
    for (auto const & ctrl : to_stop_names)
      switch_ctrl_srv.request.stop_controllers .push_back(ctrl);
  }

  // switch controllers
  if (switch_ctrl_srv.request.start_controllers.size() == 0 && switch_ctrl_srv.request.stop_controllers.size() == 0)
  {
    CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
  }

  if (!switchRequest(switch_ctrl_srv, error_, watchdog))
  {
    CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
  }
  if (!switch_ctrl_srv.response.ok)
  {
    error_ = "The service '" + switchControllerServiceName() + "' failed. Abort.";
    CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
  }


  if (watchdog.toSec() > 0)
  {
    // check the param, and exit only if the state is running!
    for (const std::string ctrl_name : switch_ctrl_srv.request.start_controllers)
    {
      // if (!cnr::control::ctrl_check_state(getNamespace(), ctrl_name, "RUNNING", error_, watchdog))
      // {
      //   CNR_RETURN_FALSE(logger_, "HW: " + getHwName() + ": " + error_);
      // }
//      if ((status != "INITIALIZED") && (status != ))
//      {
//        error_ += "Failed while checking '"+ctrl_name +"' state. The state is "+status+" while RUNNING is expected";
//        error_ +=" (transition waited for " + std::to_string( (a-n).toSec()  ) + "s, watchdog: "+ std::to_string(watchdog.toSec())+ "s)";
//        CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
//      }
    }
    for (const std::string ctrl_name : switch_ctrl_srv.request.stop_controllers)
    {
      // if (!cnr::control::ctrl_check_state(getNamespace(), ctrl_name, "STOPPED", error_, watchdog))
      // {
      //   CNR_RETURN_FALSE(logger_, "HW: " + getHwName() + ": " + error_);
      // }
    }
  }
  CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
}


bool ControllerManagerInterfaceSrv::unloadController(const std::string& ctrl_to_unload_name, const ros::Duration& watchdog)
{
  bool ret = true;
  CNR_TRACE_START(logger_, "HW: " + getHwName() + ", CTRL: " + ctrl_to_unload_name);
  controller_manager_msgs::UnloadController  unloadController_srv;
  unloadController_srv.request.name = ctrl_to_unload_name;
  if (!unloadRequest(unloadController_srv, error_, watchdog))
  {
    ret = false;
    CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
  }
  else if (!unloadController_srv.response.ok)
  {
    error_ = "The service '" + unloadServiceName() + "' failed. Abort.";
    ret = false;
    CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
  }

  std::string st = (ret  ? "UNLOADED" : "ERROR_UNLOAD");
  std::vector<std::string> status_history;
  //ros::param::get(cnr::control::ctrl_status_param_name(getHwName(), ctrl_to_unload_name),  status_history);

  status_history.push_back(st);
  //ros::param::set(cnr::control::ctrl_status_param_name(getHwName(), ctrl_to_unload_name),  status_history);
  //ros::param::set(cnr::control::ctrl_last_status_param_name(getHwName(), ctrl_to_unload_name), st) ;

  CNR_RETURN_BOOL(logger_, ret, "HW: " + getHwName() + ", CTRL: " + ctrl_to_unload_name);
}



}  // namespace cnr_controller_manager_interface

