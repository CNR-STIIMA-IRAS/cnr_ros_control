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
#include <exception>

#include <controller_manager_msgs/ListControllers.h>
#include <cnr_controller_interface_params/cnr_controller_interface_params.h>
#include <cnr_controller_manager_interface/internal/cnr_controller_manager_interface_base.h>

namespace cnr_controller_manager_interface
{


ControllerManagerInterfaceBase::ControllerManagerInterfaceBase(const cnr_logger::TraceLoggerPtr& log,
                                             const std::string& hw_name)
: nh_("/" + hw_name), logger_(log)
{
  list_       = nh_.serviceClient<controller_manager_msgs::ListControllers> ("/" + hw_name + "/controller_manager/list_controllers");
  list_types_ = nh_.serviceClient<controller_manager_msgs::ListControllerTypes> ("/" + hw_name + "/controller_manager/list_controller_types");
  assert(logger_);
}

ControllerManagerInterfaceBase::~ControllerManagerInterfaceBase()
{
  list_.shutdown();
  list_types_.shutdown();
}

bool ControllerManagerInterfaceBase::listRequest(controller_manager_msgs::ListControllers& msg,
                                             std::string& error,
                                             const ros::Duration& watchdog)
{
  return callRequest(mtx_, list_, msg, error, watchdog);
}
bool ControllerManagerInterfaceBase:: listTypeRequest(controller_manager_msgs::ListControllerTypes& msg,
                                                  std::string& error,
                                                  const ros::Duration& watchdog)
{
  return callRequest(mtx_, list_types_, msg, error, watchdog);
}

bool ControllerManagerInterfaceBase::listControllers(std::vector< controller_manager_msgs::ControllerState >&  running,
    std::vector< controller_manager_msgs::ControllerState >&  stopped,
    const ros::Duration&                                      watchdog)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(logger_);
  controller_manager_msgs::ListControllers listControllerss_srv;
  if(!callRequest(mtx_, list_, listControllerss_srv, error_, watchdog))
  {
    CNR_RETURN_FALSE(logger_, "HW: " + getHwName()  + ", "  + error_);
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
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(logger_);
}

// bool ControllerManagerInterfaceBase::matchControllers(const std::vector<std::string>& ctrl_names,
//                                                       const ros::Duration& watchdog)
// {
//   CNR_TRACE_START(logger_, "HW: " + getHwName() );
//   std::vector<controller_manager_msgs::ControllerState> running;
//   std::vector<controller_manager_msgs::ControllerState> stopped;

//   if (!listControllers(running, stopped, watchdog))
//   {
//     error_ = "Failed in getting the controllers info: " + error_;
//     CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
//   }
//   std::vector<std::string> running_names = cnr::control::ctrl_get_names(running);
//   std::vector<std::string> stopped_names = cnr::control::ctrl_get_names(stopped);
//   running_names.insert(running_names.end(), stopped_names.begin(), stopped_names.end());

//   if (ctrl_names.size() != running_names.size())
//   {
//     CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
//   }
//   for (auto const l : ctrl_names)
//   {
//     if (std::find(running_names.begin(), running_names.end(), l) == running_names.end()) return false;
//   }
//   CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
// }

bool ControllerManagerInterfaceBase::loadControllers(const std::vector<std::string>& ctrl_to_load_names, const ros::Duration& watchdog)
{
  CNR_TRACE_START(logger_, "HW: " + getHwName() );
  for (const std::string& ctrl : ctrl_to_load_names)
  {
    if (!loadController(ctrl, watchdog))
    {
      CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
    }
  }
  CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
}

bool ControllerManagerInterfaceBase::switchControllers(const std::vector<controller_manager_msgs::ControllerState>&  start_names,
                                              const std::vector<controller_manager_msgs::ControllerState>&  stop_names,
                                              const int&                                                    strictness,
                                              const ros::Duration&                                          watchdog)
{
  CNR_TRACE_START(logger_, "HW: " + getHwName() );
  std::vector<std::string>  to_start_names(start_names.size());
  std::vector<std::string>  to_stop_names(stop_names.size());

  if (to_start_names.size())
  {
    std::transform(start_names.begin(),
                   start_names.end(),
                   to_start_names.begin(),
                   [](auto v)
                      {
                        return v.name;
                      });
  }
  
  if (to_stop_names.size())
  {
    std::transform(stop_names.begin(),
                   stop_names .end(),
                   to_stop_names.begin(),
                   [](auto v)
                    {
                      return v.name;
                    });
  }
  bool ret = switchController(to_start_names, to_stop_names,strictness, watchdog);
  CNR_RETURN_BOOL(logger_, ret, "HW: " + getHwName());
}

bool ControllerManagerInterfaceBase::switchControllers(const int strictness,
                                              const std::vector<std::string>& next_ctrl,
                                              const ros::Duration& watchdog)
{
  CNR_TRACE_START(logger_, "HW: " + getHwName());
  try
  {
    std::vector< controller_manager_msgs::ControllerState > ctrl_running;
    std::vector< controller_manager_msgs::ControllerState > ctrl_stopped;

    if (!listControllers(ctrl_running, ctrl_stopped))
    {
      CNR_RETURN_FALSE(logger_, "HW: " + getHwName() + " Getting the controller info failed: " + error_);
    }

    std::vector<std::string> ctrl_running_names = cnr::control::ctrl_get_names(ctrl_running);
    std::vector<std::string> ctrl_stopped_names = cnr::control::ctrl_get_names(ctrl_stopped);
    std::vector<std::string> ctrl_loaded_names  = ctrl_running_names;
    ctrl_loaded_names.insert(ctrl_loaded_names.end(), ctrl_stopped_names.begin(), ctrl_stopped_names.end());
    std::vector<std::string> to_load_and_start_names;

    std::vector<std::string> ctrl_to_stop_and_restart_names;
    std::vector<std::string> to_stop_unload_names;
    std::vector<std::string> to_restart_names;
    std::vector<std::string> ctrl_to_unload_names;

    std::vector<std::string> ctrl_next_names = next_ctrl;

    extract(ctrl_next_names, ctrl_running_names, &to_load_and_start_names, &to_stop_unload_names, &ctrl_to_stop_and_restart_names);
    //--
    CNR_DEBUG(logger_, "HW: " + getHwName() + to_string(ctrl_next_names,                ", Next Controllers:    "));
    CNR_DEBUG(logger_, "HW: " + getHwName() + to_string(ctrl_running_names,             ", Running Controllers: "));
    CNR_DEBUG(logger_, "HW: " + getHwName() + to_string(ctrl_stopped_names,             ", Stopped Controllers: "));
    CNR_DEBUG(logger_, "HW: " + getHwName() + to_string(ctrl_loaded_names,              ", Loaded Controllers:  "));
    CNR_DEBUG(logger_, "HW: " + getHwName() + to_string(to_stop_unload_names,           ", To Stop and Unload:  "));
    CNR_DEBUG(logger_, "HW: " + getHwName() + to_string(ctrl_to_stop_and_restart_names, ", To Stop and Restart: "));

    std::vector<std::string> to_load_and_start_names_;
    extract(to_load_and_start_names, ctrl_stopped_names, &to_load_and_start_names_, &ctrl_to_unload_names, &to_restart_names);
    to_load_and_start_names = to_load_and_start_names_;

    to_restart_names.insert(to_restart_names.begin(), ctrl_to_stop_and_restart_names.begin(), ctrl_to_stop_and_restart_names.end());
    ctrl_to_unload_names .insert(ctrl_to_unload_names.begin(),  to_stop_unload_names.begin(),  to_stop_unload_names.end());

    //--
    CNR_DEBUG(logger_, "HW: "+getHwName() +to_string(to_load_and_start_names, ", To load and Start:     "));
    CNR_DEBUG(logger_, "HW: "+getHwName() +to_string(ctrl_to_unload_names,    ", To Unload Controllers: "));
    CNR_DEBUG(logger_, "HW: "+getHwName() +to_string(to_restart_names,        ", To Restart Controllers "));

    if(to_load_and_start_names.size()>0)
    {
      CNR_DEBUG(logger_, "LOAD CONTROLLERS....");
      if (!loadControllers(to_load_and_start_names, watchdog))
      {
        CNR_RETURN_FALSE(logger_, "Loading the controllers for HW '" + getHwName() + "' failed. Error: " + error());
      }
    }

    if(to_load_and_start_names.size()>0 || to_restart_names.size()>0 || to_stop_unload_names.size()>0 )
    {
      CNR_DEBUG(logger_, "SWITCH CONTROLLERS....");
      to_load_and_start_names.insert(to_load_and_start_names.end(), std::make_move_iterator(to_restart_names.begin()),
                                        std::make_move_iterator(to_restart_names.end()));
      if (!switchController(to_load_and_start_names, to_stop_unload_names, strictness,  watchdog))
      {
        CNR_RETURN_FALSE(logger_, "Switching the controllers '" + getHwName() + "'failed. Error: " + error());
      }
    }

    if(ctrl_to_unload_names.size()>0)
    {
      CNR_DEBUG(logger_, "UNLOAD CONTROLLERS....");
      if (!unloadControllers(ctrl_to_unload_names, watchdog))
      {
        CNR_RETURN_FALSE(logger_, "Unload the controllers '" + getHwName() + "'failed. Error: " + error());
      }
    }
  }
  catch(std::exception& e)
  {
    CNR_RETURN_FALSE(logger_, "Exception in switch controllers. Error: " + std::string(e.what()) );
  }
  catch(...)
  {
    CNR_RETURN_FALSE(logger_, "Unhandled Exception in switch controllers. ");
  }
  CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
}

bool ControllerManagerInterfaceBase::unloadControllers(const std::vector<std::string>& ctrl_to_unload_names,
                                              const ros::Duration& watchdog)
{
  CNR_TRACE_START(logger_, "HW: " + getHwName());
  std::vector< controller_manager_msgs::ControllerState >  running;
  std::vector< controller_manager_msgs::ControllerState >  stopped;
  if (!listControllers(running, stopped, watchdog))
  {
    CNR_RETURN_FALSE(logger_, "HW: " + getHwName()+ ", "+ error());
  }

  std::vector< std::string > rr = cnr::control::ctrl_get_names(running);
  std::vector< std::string > ss = cnr::control::ctrl_get_names(stopped);
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
      CNR_ERROR(logger_, "It is not possibile to unloade controller, since there still running controllers");
      CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
    }
  }

  for (const auto s : ss)
  {
    if (std::find(ctrl_to_unload_names.begin(), ctrl_to_unload_names.end(), s) != ctrl_to_unload_names.end())
    {
      if (!unloadController(s, watchdog))
      {
        CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
      }
    }
  }

  CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
}

bool ControllerManagerInterfaceBase::unloadControllers(const std::vector<controller_manager_msgs::ControllerState>& ctrl_to_unload_names,
                                              const ros::Duration& watchdog)
{
  CNR_TRACE_START(logger_, "HW: " + getHwName());
  std::vector<std::string> _ctrl_to_unload_names(ctrl_to_unload_names.size()) ;
  std::transform(ctrl_to_unload_names.begin(), ctrl_to_unload_names.end(), _ctrl_to_unload_names.begin(), [](auto v)
  {
    return v.name;
  });

  bool ret = unloadControllers(_ctrl_to_unload_names, watchdog);
  CNR_RETURN_BOOL(logger_, ret, "HW: " + getHwName());
}

bool ControllerManagerInterfaceBase::stopUnloadControllers(const std::vector<std::string>&  to_stop_unload_names,
                                                  const ros::Duration&  watchdog)
{
  static const std::vector<std::string> vs_empty;
  CNR_TRACE_START(logger_, "HW: " + getHwName());
  if (!switchController(vs_empty, to_stop_unload_names, 1, watchdog))
  {
    CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
  }

  bool ret = unloadControllers(to_stop_unload_names, watchdog);
  CNR_RETURN_BOOL(logger_, ret, "HW: " + getHwName());
}

bool ControllerManagerInterfaceBase::stopUnloadAllControllers(const ros::Duration&  watchdog)
{
  static const std::vector<std::string> vs_empty;
  CNR_TRACE_START(logger_, "HW: " + getHwName());
  std::vector< controller_manager_msgs::ControllerState >  stopped;

  do
  {
    std::vector< controller_manager_msgs::ControllerState >  running;

    if(!listControllers(running, stopped, watchdog))
    {
      CNR_RETURN_FALSE(logger_, "HW: " + getHwName());
    }

    if( running.size() > 0 )
    {
      std::vector<std::string> ctrls_running = cnr::control::ctrl_get_names(running);
      CNR_DEBUG(logger_, "HW: " << getHwName() << " Try to stop " << to_string(ctrls_running));
      if(!switchController(vs_empty, ctrls_running, 1, watchdog) )
      {
        CNR_RETURN_FALSE(logger_, "HW: " + getHwName() + " Error in stopping controllers");
      }
    }
    else
    {
      break;
    }
  } while( ros::ok() );

  std::vector<std::string> ctrls_stopped = cnr::control::ctrl_get_names(stopped);

  CNR_DEBUG(logger_, "HW: " << getHwName() << " Try to unload the stopped controllers: " << to_string(ctrls_stopped));
  bool ret = unloadControllers(ctrls_stopped, watchdog);
  CNR_RETURN_BOOL(logger_, ret, "HW: " + getHwName());
}



}  // namespace cnr_controller_manager_interface


