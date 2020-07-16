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

namespace cnr_controller_manager_interface
{


/**
 * @brief The ControllerManagerBase class is an abstract class that make easier the interface with
 * the controller_manager::ControllerManager
 * Two inherited classes are implemented.
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




public:  // PURE VIRTUAL METHODS
  /**
   * @brief loadController
   * @param to_load_name
   * @param watchdog
   * @return
   */
  virtual bool loadController(const std::string& to_load_name, const ros::Duration& watchdog) { return true; }

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


public:
  /**
   * @brief loadController
   * @param to_load_names
   * @param watchdog
   * @return
   */
  bool loadControllers(const std::vector<std::string>& to_load_names, const ros::Duration& watchdog=ros::Duration(0.0));

  /**
   * @brief switchController
   * @param strictness
   * @param to_load_and_start_names
   * @param to_restart_names
   * @param to_stop_unload_names
   * @param watchdog
   * @return
   */
  bool switchControllers(const int&                                                    strictness                   ,
                         const std::vector<controller_manager_msgs::ControllerState>*  to_load_and_start_names      ,
                         const std::vector<controller_manager_msgs::ControllerState>*  to_restart_names             ,
                         const std::vector<controller_manager_msgs::ControllerState>*  to_stop_unload_names         ,
                         const ros::Duration&                                          watchdog = ros::Duration(0.0));

  /**
   * @brief switchController
   * @param strictness
   * @param next_ctrl
   * @param watchdog
   * @return
   */
  bool switchControllers(const int                              strictness,
                         const std::vector<std::string> * const next_ctrl,
                         const ros::Duration&                   watchdog = ros::Duration(0.0));

  /**
   * @brief unloadController
   * @param to_unload_names
   * @param watchdog
   * @return
   */
  bool unloadControllers(const std::vector<std::string>& to_unload_names,
                         const ros::Duration& watchdog = ros::Duration(0.0));

  /**
   * @brief unloadController
   * @param to_unload_names
   * @param watchdog
   * @return
   */
  bool unloadControllers(const std::vector<controller_manager_msgs::ControllerState>& to_unload_names,
                         const ros::Duration& watchdog = ros::Duration(0.0));

  /**
   * @brief stopUnloadControllers
   * @param ctrl_to_stop_unload_names
   * @param watchdog
   * @return
   */
  bool stopUnloadControllers(const std::vector<std::string>&  ctrl_to_stop_unload_names,
                             const ros::Duration& watchdog = ros::Duration(0.0));

  /**
   * @brief stopUnloadAllControllers
   * @param watchdog
   * @return
   */
  bool stopUnloadAllControllers(const ros::Duration& watchdog = ros::Duration(0.0));

  /**
   * @brief listRequest
   * @param msg
   * @param error
   * @param watchdog
   * @return
   */
  bool listRequest(controller_manager_msgs::ListControllers& msg,
                   std::string& error,
                   const ros::Duration& watchdog = ros::Duration(0.0));

  /**
   * @brief listTypeRequest
   * @param msg
   * @param error
   * @param watchdog
   * @return
   */
  bool listTypeRequest(controller_manager_msgs::ListControllerTypes& msg,
                       std::string& error,
                       const ros::Duration& watchdog = ros::Duration(0.0));

  /**
   * @brief listControllers
   * @param running
   * @param stopped
   * @param watchdog
   * @return
   */
  bool listControllers(std::vector< controller_manager_msgs::ControllerState >&  running,
                       std::vector< controller_manager_msgs::ControllerState >&  stopped,
                       const ros::Duration&                                      watchdog = ros::Duration(0.0));

  /**
   * @brief matchControllers
   * @param names
   * @param watchdog
   * @return
   */
  bool matchControllers(const std::vector<std::string>& names, const ros::Duration& watchdog = ros::Duration(0.0));
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
};



/**
 * @brief The ControllerManagerInterface class implments the interface towards controller_manager::ControllerManager
 * using services
 */
class ControllerManagerInterface : public ControllerManagerBase
{
private:
  ros::ServiceClient                       load_;
  ros::ServiceClient                       unload_;
  ros::ServiceClient                       doswitch_;

  bool loadRequest(controller_manager_msgs::LoadController& msg,
                   std::string& error,
                   const ros::Duration&  watchdog = ros::Duration(0.0));

  bool unloadRequest(controller_manager_msgs::UnloadController& msg,
                     std::string& error,
                     const ros::Duration&  watchdog = ros::Duration(0.0));

  bool switchRequest(controller_manager_msgs::SwitchController& msg,
                     std::string& error,
                     const ros::Duration&  watchdog = ros::Duration(0.0));

public:
  ControllerManagerInterface(std::shared_ptr<cnr_logger::TraceLogger> log, const std::string& hw_name);

  std::string  loadServiceName()             { return load_.getService();     }
  std::string  unloadServiceName()           { return unload_.getService();   }
  std::string  switchControllerServiceName() { return doswitch_.getService(); }

  bool loadController(const std::string& to_load_name, const ros::Duration& watchdog);

  bool switchController(const int                       strictness,
                       const std::vector<std::string>*  to_load_and_start_names,
                       const std::vector<std::string>*  to_restart_names,
                       const std::vector<std::string>*  to_stop_unload_names,
                       const ros::Duration&             watchdog = ros::Duration(0.0));

  bool unloadController(const std::string& to_unload_name, const ros::Duration& watchdog = ros::Duration(0.0));
};




/**
 * @brief The ControllerManager class integrates an instance of the controller_manager::ControllerManager
 */
class ControllerManager : public ControllerManagerBase
{
private:
  controller_manager::ControllerManager cm_;

public:
  ControllerManager(std::shared_ptr<cnr_logger::TraceLogger>  log,
                    const std::string&                        hw_name,
                    hardware_interface::RobotHW*              robot_hw,
                    const ros::NodeHandle&                    nh=ros::NodeHandle());

  bool loadController(const std::string&   to_load_name,
                      const ros::Duration& watchdog = ros::Duration(0.0));

  bool switchController(const int                        strictness,
                        const std::vector<std::string>*  to_load_and_start_names,
                        const std::vector<std::string>*  to_restart_names,
                        const std::vector<std::string>*  to_stop_unload_names,
                        const ros::Duration&             watchdog = ros::Duration(0.0));

  bool unloadController(const std::string& to_unload_name, const ros::Duration& watchdog = ros::Duration(0.0));

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

#endif  // CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER_INTERFACE_H
