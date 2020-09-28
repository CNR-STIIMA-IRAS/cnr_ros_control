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
#ifndef CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER_BASE_H
#define CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER_BASE_H

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

namespace cnr_controller_manager_interface
{


/**
 * @brief The ControllerManagerBase class is a class that make easier the interface with
 * the controller_manager::ControllerManager (http://wiki.ros.org/ros_control)
 *
 * The class implement the base interface to access to the standard functionalities of the
 * controller_manager::ControllerManager.
 * It is a collection of fake-methods that "echo" the methods of the standard controller_manager::ControllerManager
 *
 * This class is inherited by three different classes:
 * A) The ControllerManager: the class stores as variable the standard controller_manager::ControllerManager, so when
 *    you call the method, you actually call the standard controller_manager::ControllerManager inside.
 *    Furthermore, the varible controller_manager::ControllerManager offers the standard services.
 * B) The ControllerManagerProxy: the class is a proxy for the ControllerManager (A). Indeed, it offers the same services
 *    of the ControllerManager, but it modifies the namespace. Therefore, if you call - as matter for example - the
 *    switchController(), it executes some fancy thigs before and after the actual switchController of the standard
 *    controller_manager::COntrollerManager. As matter of example, it offers diganostics, it dump the status of the
 *    controllers on the ros parameters etc.
 * C) The ControllerManagerInterface: it just echos the request to the corrisponding service provided by the
 *    controller_manager::ControllerManager that is supposed to run in a different node/nodelet.
 *    This class should be used by the ConfigurationManager, since it cannot access directly to the controller manager,
 *    Indeed, the controller manager runs inside the nodelet corresponding to the RobotHw, while the configuration
 *    manager runs in a node (that creates the robot hw nodelet) *
 * The only service actually implemented is the ListRequest.
 * Indeed, this service cannot be accessed by the controller_manager::ControllerManager class thorugh a method
 * On the contrary, all the other services can be accessed alternatively or by a remote serice or by calling a method
 */
class ControllerManagerBase
{
protected:
  ros::NodeHandle                          nh_;

  ros::ServiceClient                       list_;
  ros::ServiceClient                       list_types_;
  std::shared_ptr<cnr_logger::TraceLogger> logger_;
  std::string                              error_;


public:
  ControllerManagerBase(std::shared_ptr<cnr_logger::TraceLogger> log, const std::string& hw_name);
  ~ControllerManagerBase();

  std::shared_ptr<cnr_logger::TraceLogger> getLogger() { return logger_;   }
  std::string error()               { return error_;   }
  std::string getHwName()           { std::string n = nh_.getNamespace(); n.erase(0, 1);  return n; }
  std::string getNamespace()        { return nh_.getNamespace();       }
  std::string listServiceName()     { return list_.getService();       }
  std::string listTypeServiceName() { return list_types_.getService(); }

// VIRTUAL METHODS //
  /**
   * @brief loadController
   * @param to_load_name
   * @param watchdog
   * @return
   */
  virtual bool loadController(const std::string& /*to_load_name*/, const ros::Duration& /*watchdog*/) { return true; }

  /**
   * @brief switchController
   * @param strictness
   * @param to_load_and_start_names
   * @param to_restart_names
   * @param to_stop_unload_names
   * @param watchdog
   * @return
   */
  virtual bool switchController (const int                        strictness                   ,
                                 const std::vector<std::string>*  to_load_and_start_names      ,
                                 const std::vector<std::string>*  to_restart_names             ,
                                 const std::vector<std::string>*  to_stop_unload_names         ,
                                 const ros::Duration&             watchdog = ros::Duration(0.0)) { return true; }

  /**
   * @brief unloadController
   * @param to_unload_name
   * @param watchdog
   * @return
   */
  virtual bool unloadController(const std::string&    to_unload_name,
                                const ros::Duration&  watchdog = ros::Duration(0.0)) { return true; }


// CORE METHODS //
  /**
   * @brief loadController
   * @param to_load_names
   * @param watchdog
   * @return
   */
  virtual bool loadControllers(const std::vector<std::string>& to_load_names,
                               const ros::Duration& watchdog=ros::Duration(0.0)) final;

  /**
   * @brief switchController
   * @param strictness
   * @param to_load_and_start_names
   * @param to_restart_names
   * @param to_stop_unload_names
   * @param watchdog
   * @return
   */
  virtual bool switchControllers(const int&                                        strictness                   ,
                     const std::vector<controller_manager_msgs::ControllerState>*  load_and_start_names      ,
                     const std::vector<controller_manager_msgs::ControllerState>*  restart_names             ,
                     const std::vector<controller_manager_msgs::ControllerState>*  stop_unload_names         ,
                     const ros::Duration&                                          watchdog = ros::Duration(0.0)
                                ) final;

  /**
   * @brief switchController
   * @param strictness
   * @param next_ctrl
   * @param watchdog
   * @return
   */
  virtual bool switchControllers(const int                              strictness,
                                 const std::vector<std::string> * const next_ctrl,
                                 const ros::Duration&                   watchdog = ros::Duration(0.0)) final;

  /**
   * @brief unloadController
   * @param to_unload_names
   * @param watchdog
   * @return
   */
  virtual bool unloadControllers(const std::vector<std::string>& to_unload_names,
                                 const ros::Duration& watchdog = ros::Duration(0.0)) final;

  /**
   * @brief unloadController
   * @param to_unload_names
   * @param watchdog
   * @return
   */
  virtual bool unloadControllers(const std::vector<controller_manager_msgs::ControllerState>& to_unload_names,
                                 const ros::Duration& watchdog = ros::Duration(0.0)) final;

  /**
   * @brief stopUnloadControllers
   * @param ctrl_to_stop_unload_names
   * @param watchdog
   * @return
   */
  virtual bool stopUnloadControllers(const std::vector<std::string>&  ctrl_to_stop_unload_names,
                                     const ros::Duration& watchdog = ros::Duration(0.0)) final;

  /**
   * @brief stopUnloadAllControllers
   * @param watchdog
   * @return
   */
  virtual bool stopUnloadAllControllers(const ros::Duration& watchdog = ros::Duration(0.0)) final;

  /**
   * @brief listRequest
   * @param msg
   * @param error
   * @param watchdog
   * @return
   */
  virtual bool listRequest(controller_manager_msgs::ListControllers& msg,
                           std::string& error,
                           const ros::Duration& watchdog = ros::Duration(0.0)) final;

  /**
   * @brief listTypeRequest
   * @param msg
   * @param error
   * @param watchdog
   * @return
   */
  virtual bool listTypeRequest(controller_manager_msgs::ListControllerTypes& msg,
                               std::string& error,
                               const ros::Duration& watchdog = ros::Duration(0.0)) final;

  /**
   * @brief listControllers
   * @param running
   * @param stopped
   * @param watchdog
   * @return
   */
  virtual bool listControllers(std::vector<controller_manager_msgs::ControllerState>&  running,
                               std::vector<controller_manager_msgs::ControllerState>&  stopped,
                               const ros::Duration&  watchdog=ros::Duration(0.0)) final;

  /**
   * @brief matchControllers
   * @param names
   * @param watchdog
   * @return
   */
  virtual bool matchControllers(const std::vector<std::string>& names,
                                const ros::Duration& watchdog = ros::Duration(0.0)) final;
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
};





}  // namespace cnr_controller_manager_interface

#endif  // CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER_BASE_H
