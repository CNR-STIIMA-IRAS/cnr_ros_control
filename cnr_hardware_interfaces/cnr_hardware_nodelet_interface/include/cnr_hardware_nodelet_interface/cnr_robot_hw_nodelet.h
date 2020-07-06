#ifndef __BASIC_HARDWARE_INTERFACE__ON_NODELET__
#define __BASIC_HARDWARE_INTERFACE__ON_NODELET__

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

  ~RobotHwNodelet( );

  virtual bool doOnInit   ( );
  bool         enterOnInit( );
  bool         exitOnInit ( );

  void         diagnosticsThread  ( );
  void         controlUpdateThread( );

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

  void diagnostics( diagnostic_updater::DiagnosticStatusWrapper &stat );

  std::shared_ptr< pluginlib::ClassLoader< cnr_hardware_interface::RobotHW > > m_robot_hw_plugin_loader;

};

}

#endif
