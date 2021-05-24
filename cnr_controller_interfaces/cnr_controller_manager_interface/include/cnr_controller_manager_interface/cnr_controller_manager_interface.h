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

#include <cnr_controller_manager_interface/internal/cnr_controller_manager_interface_base.h>

namespace cnr_controller_manager_interface
{



/**
 * @brief The ControllerManagerInterface integrates an instance of the controller_manager::ControllerManagerInterface
 *
 * The class load/unload the controllers without the use of service, but just calling the proper function.
 * The class can be seen as a "pImpl" (https://en.cppreference.com/w/cpp/language/pimpl) for the
 * standard controller_manager::ControllerManagerInterface. Furthermore, the inheritance from
 * cnr_controller_manager_interface::ControllerManagerBase allows to have a standard interface for passing the request
 * through vectors of names.
 *
 */
class ControllerManagerInterface : public ControllerManagerInterfaceBase
{
protected:
  controller_manager::ControllerManager* cm_;
  std::map<std::string, controller_interface::ControllerBase* > controllers_;

public:
  typedef std::shared_ptr<ControllerManagerInterface> Ptr;
  typedef std::shared_ptr<ControllerManagerInterface const> ConstPtr;
  
  ControllerManagerInterface() = delete;
  virtual ~ControllerManagerInterface();
  ControllerManagerInterface(const ControllerManagerInterface&) = delete;
  ControllerManagerInterface& operator=(const ControllerManagerInterface&) = delete;
  ControllerManagerInterface(ControllerManagerInterface&&) = delete;
  ControllerManagerInterface& operator=(ControllerManagerInterface&&) = delete;
  
  ControllerManagerInterface(const cnr_logger::TraceLoggerPtr& log,
                    const std::string&                hw_name,
                    controller_manager::ControllerManager* cm);

  /** \name Non Real-Time Safe Functions
   *\{*/

  /** \brief Load a new controller by name. Wrap of ControllerManagerInterface::loadController()
   *
   * This dynamically loads a controller called \c name and initializes the
   * newly
   * loaded controller.
   *
   * It determines the controller type by accessing the ROS parameter "type" in
   * the namespace given by \c name relative to the namespace of \ref
   * root_nh_. It then initializes the controller with the
   * hardware_interface::RobotHW pointer \ref robot_hw_, the ros::NodeHandle
   * describing this namespace, and a reference to a std::set to retrieve the
   * resources needed by this controller.
   *
   * A controller cannot be loaded while already loaded. To re-load a
   * controller, first \ref unloadController and then \ref loadController.
   *
   * \param name The name of the controller as well as the ROS namespace under
   * which the controller should be loaded
   *
   * \returns True on success
   * \returns False on failure
   */
  bool loadController(const std::string& to_load_name,
                      const ros::Duration& watchdog = ros::Duration(0.0)) final;

 /** \brief Unload a controller by name. Wrap of ControllerManagerInterface::unloadController()
   *
   * \param name The name of the controller to unload. (The same as the one used in \ref loadController )
   *
   */
  bool unloadController(const std::string& to_unload_name,
                        const ros::Duration& watchdog = ros::Duration(0.0)) final;

  /** \brief Switch multiple controllers simultaneously. Wrap of ControllerManagerInterface::switchController()
   *
   * \param start_controllers A vector of controller names to be started
   * \param stop_controllers A vector of controller names to be stopped
   * \param strictness How important it is that the requested controllers are
   * started and stopped.  The levels are defined in the
   * controller_manager_msgs/SwitchControllers service as either \c BEST_EFFORT
   * or \c STRICT.  \c BEST_EFFORT means that \ref switchController can still
   * succeed if a non-existent controller is requested to be stopped or started.
   * \param start_asap Start the controllers as soon as their resources
   * are ready, will wait for all resources to be ready otherwise.
   * \param timeout The timeout in seconds before aborting pending
   * controllers. Zero for infinite.
   */
  bool switchController(const std::vector<std::string>&  to_start_names,
                        const std::vector<std::string>&  to_stop_unload_names, 
                        const int                        strictness,
                        const ros::Duration&             watchdog = ros::Duration(0.0)) final;

  
  /**
   * @brief listControllers
   * @param running
   * @param stopped
   * @param watchdog
   * @return
   */
  bool listControllers(std::vector<controller_manager_msgs::ControllerState>&  running,
                       std::vector<controller_manager_msgs::ControllerState>&  stopped,
                       const ros::Duration&  watchdog=ros::Duration(0.0));
  
  
  /** \brief Get a controller by name. Wrap of ControllerManagerInterface::getControllerByName()
   *
   * \param name The name of a controller
   * \returns An up-casted pointer to the controller identified by \c name
   */
  controller_interface::ControllerBase* getControllerByName(const std::string& name)
  {
    return cm_ ? cm_->getControllerByName(name) : nullptr;
  }
  /*\}*/


  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Update all active controllers. Wrap of ControllerManagerInterface::update()
   *
   * When controllers are started or stopped (or switched), those calls are
   * made in this function.
   *
   * \param time The current time
   * \param period The change in time since the last call to \ref update
   * \param reset_controllers If \c true, stop and start all running
   * controllers before updating
   */
  void update(const ros::Time& time, const ros::Duration& period, bool reset_controllers=false)
  {
    return cm_->update(time,period,reset_controllers);
  }
  /*\}*/
  
  /** \name Non Real-Time Safe Functions
   *\{*/
  /** \brief Logging functions compliant to DiagnositcUpdater framework
   * 
   * In the case the controller inherits from cnr_controller_interface::Controller<>, it
   * provides also an inheritance from the class 'realtime_utilities::DiagnosticsInterface'
   * In such a case, the function ectract the controller logging informantion
   * and it appends them to the 'stat' variable
   *  
   * \param[in/out] stat see DiagnosticUpdater
   */
  void diagnosticsInfo(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void diagnosticsWarn(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void diagnosticsError(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void diagnosticsPerformance(diagnostic_updater::DiagnosticStatusWrapper &stat);
  /*\}*/

};

typedef ControllerManagerInterface::Ptr ControllerManagerInterfacePtr;
typedef ControllerManagerInterface::ConstPtr ControllerManagerInterfaceConstPtr;

}  // namespace cnr_controller_manager_interface

#endif  // CNR_CONTROLLER_MANAGER_INTERFACE_CNR_CONTROLLER_MANAGER__H
