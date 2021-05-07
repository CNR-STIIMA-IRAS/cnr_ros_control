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
#ifndef CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER__H
#define CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER__H

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
 * @brief The ControllerManager integrates an instance of the controller_manager::ControllerManager
 *
 * The class load/unload the controllers without the use of service, but just calling the proper function.
 * The class can be seen as a "pImpl" (https://en.cppreference.com/w/cpp/language/pimpl) for the
 * standard controller_manager::ControllerManager. Furthermore, the inheritance from
 * cnr_controller_manager_interface::ControllerManagerBase allows to have a standard interface for passing the request
 * through vectors of names.
 *
 */
class ControllerManager : public ControllerManagerBase
{
protected:
  controller_manager::ControllerManager cm_;

public:
  
  ControllerManager() = delete;
  virtual ~ControllerManager() = default;
  ControllerManager(const ControllerManager&) = delete;
  ControllerManager& operator=(const ControllerManager&) = delete;
  ControllerManager(ControllerManager&&) = delete;
  ControllerManager& operator=(ControllerManager&&) = delete;
  
  ControllerManager(std::shared_ptr<cnr_logger::TraceLogger>  log,
                    const std::string&                        hw_name,
                    hardware_interface::RobotHW*              robot_hw,
                    const ros::NodeHandle&                    nh=ros::NodeHandle());

  bool loadController(const std::string&   to_load_name,
                      const ros::Duration& watchdog = ros::Duration(0.0)) final;

  bool switchController(const int                        strictness,
                        const std::vector<std::string>&  to_load_and_start_names,
                        const std::vector<std::string>&  to_restart_names,
                        const std::vector<std::string>&  to_stop_unload_names,
                        const ros::Duration&             watchdog = ros::Duration(0.0) ) final;

  bool unloadController(const std::string& to_unload_name,
                        const ros::Duration& watchdog = ros::Duration(0.0)) final;

  controller_interface::ControllerBase* getControllerByName(const std::string& name)
  {
    return cm_.getControllerByName(name);
  }

  void update(const ros::Time& time, const ros::Duration& period, bool reset_controllers=false)
  {
    return cm_.update(time,period,reset_controllers);
  }
};

}  // namespace cnr_controller_manager_interface

#endif  // CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER__H
