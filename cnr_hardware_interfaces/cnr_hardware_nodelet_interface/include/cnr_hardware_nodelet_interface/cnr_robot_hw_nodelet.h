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
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletList.h>
#include <pluginlib/class_loader.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <realtime_utilities/circular_buffer.h>
#include <cnr_logger/cnr_logger.h>
#include <controller_manager/controller_manager.h>
#include <cnr_controller_manager_interface/cnr_controller_manager_interface.h>
#include <cnr_hardware_interface/cnr_robot_hw.h>

namespace cnr_hardware_nodelet_interface
{


class RobotHwNodelet : public nodelet::Nodelet
{
public:
  void onInit() final;

  RobotHwNodelet();
protected:

  ~RobotHwNodelet();

  virtual bool doOnInit();
  bool         enterOnInit();
  bool         exitOnInit();

  void         diagnosticsThread();
  void         controlUpdateThread();

  enum THREAD_STATE { ON_INIT, RUNNING, ON_ERROR, EXPIRED } m_diagnostics_thread_state, m_update_thread_state;

  std::thread m_diagnostics_thread, m_update_thread;

  bool                                                                  m_stop_update_thread;
  bool                                                                  m_stop_diagnostic_thread;
  ros::NodeHandle                                                       m_nh;
  ros::NodeHandle                                                       m_hw_nh;
  boost::shared_ptr< cnr_hardware_interface::RobotHW >                  m_hw;
  std::string                                                           m_hw_namespace;
  std::string                                                           m_hw_name;
  std::shared_ptr<controller_manager::ControllerManager>                m_cm;
  ros::Duration                                                         m_period;

  std::map<std::string, realtime_utilities::TimeSpanTracker* >          m_time_span_tracker;
  diagnostic_updater::Updater                                           m_updater;
  std::shared_ptr<cnr_logger::TraceLogger>                              m_logger;

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  std::shared_ptr< pluginlib::ClassLoader< cnr_hardware_interface::RobotHW > > m_robot_hw_plugin_loader;

};

}  // namepsace cnr_hardware_nodelet_interface

#endif  // CNR_HARDWARE_NODELET_INTERFACE_CNR_ROBOT_HW_NODELET_H
