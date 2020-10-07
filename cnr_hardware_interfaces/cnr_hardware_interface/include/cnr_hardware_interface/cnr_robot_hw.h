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

#ifndef  CNR_HARDWARE_INTERFACE_CNR_ROBOT_HW_H
#define  CNR_HARDWARE_INTERFACE_CNR_ROBOT_HW_H

#include <string>
#include <list>
#include <vector>
#include <map>
#include <mutex>  // NOLINT
#include <functional>
#include <thread>  // NOLINT

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <cnr_logger/cnr_logger.h>

#include <hardware_interface/robot_hw.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <configuration_msgs/SetConfig.h>
#include <configuration_msgs/GetConfig.h>
//#include <cnr_controller_interface/cnr_controller_interface.h>
//#include <cnr_controller_manager_interface/cnr_controller_manager_interface.h>
#include <cnr_hardware_interface/internal/cnr_robot_hw_status.h>
#include <cnr_hardware_interface/internal/cnr_robot_hw_utils.h>


namespace cnr_hardware_interface
{

typedef std::function<void(const std::string&)> SetStatusParamFcn;

inline std::string extractRobotName(const std::string& hw_namespace)
{
  std::string hw_name = hw_namespace;
  hw_name      .erase(0, 1);
  std::replace(hw_name.begin(), hw_name.end(), '/', '_');
  return hw_name;
}

/**
 * @brief The RobotHW class
 */
class RobotHW: public hardware_interface::RobotHW
{
public:
  RobotHW();
  ~RobotHW();

  // ======================================================= final methods (cannot be overriden by the derived clases
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) final;
  void read(const ros::Time& time, const ros::Duration& period) final;
  void write(const ros::Time& time, const ros::Duration& period) final;
  bool prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
                     const std::list< hardware_interface::ControllerInfo >& stop_list) final;
  bool checkForConflict(const std::list< hardware_interface::ControllerInfo >& info) const final;
  bool shutdown();
  // ======================================================= End - final methods



  // ======================================================= Method to override inthe derived classes
  virtual bool doInit()
  {
    return true;
  }
  virtual bool doShutdown()
  {
    return true;
  }
  virtual bool doRead(const ros::Time& time, const ros::Duration& period)
  {
    return true;
  }
  virtual bool doWrite(const ros::Time& time, const ros::Duration& period)
  {
    return true;
  }
  virtual bool doPrepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
                               const std::list< hardware_interface::ControllerInfo >& stop_list)
  {
    return true;
  }
  virtual bool doCheckForConflict(const std::list< hardware_interface::ControllerInfo >& info) const
  {
    return false;
  }
  // ======================================================= END - Method to override inthe derived classes


  // =======================================================
  void setResourceNames(const std::vector<std::string>& resource_names)
  {
    m_resource_names = resource_names;
  }
  // =======================================================

  // ======================================================= utils
  void diagnostics     (diagnostic_updater::DiagnosticStatusWrapper &stat, int level);
  void diagnosticsInfo (diagnostic_updater::DiagnosticStatusWrapper &stat);
  void diagnosticsWarn (diagnostic_updater::DiagnosticStatusWrapper &stat);
  void diagnosticsError(diagnostic_updater::DiagnosticStatusWrapper &stat);

  const cnr_hardware_interface::StatusHw&  getStatus() const
  {
    return m_status;
  }
  const std::string& getRobotHwNamespace() const
  {
    return m_robothw_nh.getNamespace();
  }
  // ======================================================= END - utils

protected:
  virtual bool setParamServer(configuration_msgs::SetConfigRequest& req, configuration_msgs::SetConfigResponse& res);
  virtual bool getParamServer(configuration_msgs::GetConfigRequest& req, configuration_msgs::GetConfigResponse& res);

  void add_diagnostic_message(const std::string& level,
                              const std::string& summary,
                              const std::map<std::string, std::string>& key_values,
                              const bool verbose = false);
  bool dump_state(const cnr_hardware_interface::StatusHw& status) const;
  bool dump_state() const;

private:
  virtual bool enterInit(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  virtual bool enterShutdown();
  virtual bool enterWrite();
  virtual bool enterPrepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
                                  const std::list< hardware_interface::ControllerInfo >& stop_list);
  virtual bool enterCheckForConflict(const std::list< hardware_interface::ControllerInfo >& info) const;

  virtual bool exitInit();
  virtual bool exitShutdown();
  virtual bool exitWrite();
  virtual bool exitPrepareSwitch();
  virtual bool exitCheckForConflict() const
  {
    return false;
  }

protected:
  std::string                                      m_robot_name;
  ros::NodeHandle                                  m_root_nh;
  ros::NodeHandle                                  m_robothw_nh;
  ros::CallbackQueue                               m_robot_hw_queue;
  std::shared_ptr<cnr_logger::TraceLogger>         m_logger;

  SetStatusParamFcn                                m_set_status_param;

  std::mutex                                       m_mutex;
  ros::ServiceServer                               m_get_param;
  ros::ServiceServer                               m_set_param;
  bool                                             m_stop_thread;

  bool                                             m_is_first_read;
  mutable cnr_hardware_interface::StatusHw         m_status;
  mutable std::vector<std::string>                 m_status_history;
  mutable diagnostic_msgs::DiagnosticArray         m_diagnostic;
  
  std::list< hardware_interface::ControllerInfo >  m_active_controllers;
  std::vector< std::string >                       m_resource_names;
  bool                                             m_shutted_down;
};

typedef std::shared_ptr<RobotHW> RobotHWSharedPtr;

}  // namespace cnr_hardware_interface

#endif  // CNR_HARDWARE_INTERFACE_CNR_ROBOT_HW_H
