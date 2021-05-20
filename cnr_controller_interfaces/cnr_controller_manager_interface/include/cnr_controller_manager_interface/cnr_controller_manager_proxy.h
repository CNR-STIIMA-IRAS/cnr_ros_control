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
#ifndef CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER_PROXY_H
#define CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER_PROXY_H

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

#include <cnr_controller_manager_interface/cnr_controller_manager.h>

namespace cnr_controller_manager_interface
{

/**
 * @brief The ControllerManagerProxy class
 *
 * The class implement the ros-style interface to access to the standard functionalities of the
 * controller_manager::ControllerManager.
 * It is a collection of fake-methods that "echo" the methods of the standard controller_manager::ControllerManager
 *
 * The class is a proxy for the ControllerManager (A). Indeed, it offers the same services
 * of the ControllerManager, but it modifies the namespace. Therefore, if you call - as matter for example - the
 * switchController(), it executes some fancy thigs before and after the actual switchController of the standard
 * controller_manager::ControllerManager. As matter of example, it offers diganostics, it dump the status of the
 * controllers on the ros parameters etc.
 */
class ControllerManagerProxy : public cnr_controller_manager_interface::ControllerManager
{
public:
  typedef std::shared_ptr<ControllerManagerProxy> Ptr;
  typedef std::shared_ptr<ControllerManagerProxy const> ConstPtr;

  ControllerManagerProxy() = delete;
  virtual ~ControllerManagerProxy() = default;
  ControllerManagerProxy(const ControllerManagerProxy&) = delete;
  ControllerManagerProxy& operator=(const ControllerManagerProxy&) = delete;
  ControllerManagerProxy(ControllerManagerProxy&&) = delete;
  ControllerManagerProxy& operator=(ControllerManagerProxy&&) = delete;
    
  ControllerManagerProxy(cnr_logger::TraceLogger*     logger,
                         const std::string&           hw_name,
                         hardware_interface::RobotHW* robot_hw,
                         const ros::NodeHandle&       nh=ros::NodeHandle());
private:
  /** \name ROS Service API
   *\{*/
  bool loadControllerSrv(controller_manager_msgs::LoadController::Request& req,
                         controller_manager_msgs::LoadController::Response& res );
  bool unloadControllerSrv(controller_manager_msgs::UnloadController::Request& req,
                           controller_manager_msgs::UnloadController::Response& res );
  bool switchControllerSrv(controller_manager_msgs::SwitchController::Request& req,
                           controller_manager_msgs::SwitchController::Response& res );

  mutable std::mutex mtx_;
  ros::ServiceServer load_;
  ros::ServiceServer unload_;
  ros::ServiceServer doswitch_;
  /*\}*/
};

typedef ControllerManagerProxy::Ptr ControllerManagerProxyPtr;
typedef ControllerManagerProxy::ConstPtr ControllerManagerProxyConstPtr;


}  // namespace cnr_controller_manager_interface

#endif  // CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER_PROXY_H
