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
#ifndef CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER_INTERFACE_H
#define CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER_INTERFACE_H

#include <mutex>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <cnr_logger/cnr_logger.h>
#include <controller_manager/controller_manager.h>
#include <cnr_controller_manager_interface/internal/utils.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

#include <cnr_controller_manager_interface/internal/cnr_controller_manager_base.h>

namespace cnr_controller_manager_interface
{


/**
 * @brief The ControllerManagerInterface class implments the interface towards controller_manager::ControllerManager
 * using services, and it adds diagnostics, and a logger. Furthermore, it set properly the namespace of the services
 * under the scheme: /hw_id/serivce_name
 *
 * It is supposed to be used by the client-application
 */
class ControllerManagerInterface : public cnr_controller_manager_interface::ControllerManagerBase
{
private:
  ros::ServiceClient load_;
  ros::ServiceClient unload_;
  ros::ServiceClient doswitch_;

  bool loadRequest(controller_manager_msgs::LoadController& msg, std::string& error,
                   const ros::Duration&  watchdog = ros::Duration(0.0));

  bool unloadRequest(controller_manager_msgs::UnloadController& msg, std::string& error,
                     const ros::Duration&  watchdog = ros::Duration(0.0));

  bool switchRequest(controller_manager_msgs::SwitchController& msg, std::string& error,
                     const ros::Duration&  watchdog = ros::Duration(0.0));

public:
  
  ControllerManagerInterface() = delete;
  virtual ~ControllerManagerInterface() = default;
  ControllerManagerInterface(const ControllerManagerInterface&) = delete;
  ControllerManagerInterface& operator=(const ControllerManagerInterface&) = delete;
  ControllerManagerInterface(ControllerManagerInterface&&) = delete;
  ControllerManagerInterface& operator=(ControllerManagerInterface&&) = delete;

  /**
   * @brief ControllerManagerInterface
   * @param logger
   * @param hw_name
   * @param use_proxy
   */
  ControllerManagerInterface(std::shared_ptr<cnr_logger::TraceLogger> logger,
                             const std::string&                       hw_name,
                             const bool&                              use_proxy = false );

  std::string  loadServiceName()             { return load_.getService();     }
  std::string  unloadServiceName()           { return unload_.getService();   }
  std::string  switchControllerServiceName() { return doswitch_.getService(); }

  bool loadController(const std::string& to_load_name, const ros::Duration& watchdog) final;

  bool switchController(const int strictness,
                        const std::vector<std::string>& to_load_and_start_names,
                        const std::vector<std::string>& to_restart_names,
                        const std::vector<std::string>& to_stop_unload_names,
                        const ros::Duration& watchdog = ros::Duration(0.0)) final;

  bool unloadController(const std::string& to_unload_name, const ros::Duration& watchdog = ros::Duration(0.0)) final;
};


typedef std::shared_ptr< ControllerManagerInterface > ControllerManagerInterfacePtr;
typedef const std::shared_ptr< ControllerManagerInterface const > ControllerManagerInterfaceConstPtr;

}  // namespace cnr_controller_manager_interface

#endif  // CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER_INTERFACE_H
