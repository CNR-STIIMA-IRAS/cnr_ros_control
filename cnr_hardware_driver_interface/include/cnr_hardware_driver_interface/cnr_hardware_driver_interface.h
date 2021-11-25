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
#ifndef CNR_HARDWARE_NODELET_INTERFACE_CNR_ROBOT_HW_NODELET_H
#define CNR_HARDWARE_NODELET_INTERFACE_CNR_ROBOT_HW_NODELET_H

#include <thread>
#include <memory>
#include <map>
#include <string>
#include <mutex>


#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <realtime_utilities/time_span_tracker.h>
#include <cnr_logger/cnr_logger.h>
#include <controller_manager/controller_manager.h>
#include <cnr_controller_manager_interface/cnr_controller_manager_proxy.h>
#include <cnr_hardware_interface/cnr_robot_hw_status.h>
#include <cnr_hardware_interface/cnr_robot_hw.h>

namespace cnr_hardware_driver_interface
{

typedef std::shared_ptr< controller_manager::ControllerManager > ControllerManagerPtr;
typedef std::shared_ptr< pluginlib::ClassLoader< hardware_interface::RobotHW > > RobotLoaderPtr;

typedef std::shared_ptr< hardware_interface::RobotHW > RobotHWPtr;
typedef cnr_hardware_interface::RobotHW* CnrRobotHWPtr;

typedef std::shared_ptr< hardware_interface::RobotHW const> RobotHWConstPtr;
typedef cnr_hardware_interface::RobotHW* const CnrRobotHWConstPtr;


class RobotHwDriverInterface : public realtime_utilities::DiagnosticsInterface
{
public:
  typedef std::shared_ptr<RobotHwDriverInterface> Ptr;
  typedef std::shared_ptr<RobotHwDriverInterface const> ConstPtr;

  RobotHwDriverInterface() = default;
  virtual ~RobotHwDriverInterface();

  bool init(const std::string& hw_name, 
              const std::map<std::string, std::string>& remappings);

  /** @brief Control Loop implementing a infinite loop [HW read()-> CM update() -> HW write()]
   * 
   * The run() function is a blocking function. To execute it in a parallel thread
   * you must call start() and stop()
   */
  void run();

  /** @brief Start the control loop implemented in 'run()' in a separate thread
   * 
   * @param[in] watchdog
   * @param[out] true/false if ok or some errors arisen.
   */
  bool start(const ros::Duration& watchdog = ros::Duration(1.0) );
  
  /** @brief Stop the control loop implemented in 'run()' if running in a separate thread
   * 
   * @param[in] watchdog
   * @param[out] true/false if ok or some errors arisen.
   */
  bool stop(const ros::Duration& watchdog = ros::Duration(1.0) );

  /** @brief get the state of the the Driver
   * 
   * The state of the driver is the same state of the RobotHW if the 
   * loaded class is inherited from cnr_hardware_interface::RobotHW
   */
  const cnr_hardware_interface::StatusHw& getState() const
  {
    return m_state;
  }

  /** @brief get the state of the the RobotHW
   * 
   * The state of the RobotHW is the state of the driver if the 
   * loaded class is inherited from cnr_hardware_interface::RobotHW
   */
  const cnr_hardware_interface::StatusHw& retriveState()
  {
    m_state = m_cnr_hw ? m_cnr_hw->getStatus() : m_state; 
    return m_state;
  }
  
  RobotHWConstPtr getRobotHw() const { return m_hw; }
  CnrRobotHWConstPtr getCnrRobotHw() const  { return m_cnr_hw; }
  
  ControllerManagerPtr getControllerManager()
  {
    return m_cm;
  }
  cnr_controller_manager_interface::ControllerManagerInterfacePtr getControllerManagerInterface()
  {
    return m_cmi;
  }

protected:

  bool dumpState(const cnr_hardware_interface::StatusHw& status);
  bool fetchState(std::string& error);

  cnr_logger::TraceLoggerPtr  m_logger;
  ros::CallbackQueue          m_callback_queue;
  ros::NodeHandle             m_root_nh;
  ros::NodeHandle             m_hw_nh;
  std::string                 m_hw_namespace;
  std::string                 m_hw_name;

  RobotHWPtr              m_hw;
  CnrRobotHWPtr           m_cnr_hw = nullptr;
  RobotLoaderPtr          m_robot_hw_loader;
  ControllerManagerPtr    m_cm;
  cnr_controller_manager_interface::ControllerManagerInterfacePtr m_cmi;
  
  mutable std::mutex                m_mtx;
  cnr_hardware_interface::StatusHw  m_state;
  std::vector<std::string>          m_state_history;
  bool                              m_stop_run;
  ros::Duration                     m_period;
  std::thread                       m_thread_run;

  bool m_diagnostics_thread_running;
  bool m_stop_diagnostic_thread;
  std::thread m_diagnostics_thread;
  void diagnosticsThread();
};

typedef RobotHwDriverInterface::Ptr RobotHwDriverInterfacePtr;
typedef RobotHwDriverInterface::ConstPtr RobotHwDriverInterfaceConstPtr;

}  // namepsace cnr_hardware_driver_interface

#endif  // CNR_HARDWARE_NODELET_INTERFACE_CNR_ROBOT_HW_NODELET_H
