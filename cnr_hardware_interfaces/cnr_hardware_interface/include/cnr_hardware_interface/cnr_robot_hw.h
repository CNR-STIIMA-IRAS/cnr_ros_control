#ifndef __CNR__ROBOT__HW__H__
#define __CNR__ROBOT__HW__H__

#include <mutex>
#include <functional>
#include <thread>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <cnr_logger/cnr_logger.h>

#include <hardware_interface/robot_hw.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <configuration_msgs/SetConfig.h>
#include <configuration_msgs/GetConfig.h>
#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_controller_manager_interface/cnr_controller_manager_interface.h>
#include <cnr_hardware_interface/internal/cnr_robot_hw_status.h>
#include <cnr_hardware_interface/internal/cnr_robot_hw_utils.h>


namespace cnr_hardware_interface
{

  typedef std::function<void(const std::string&)> SetParamFcn;

  inline std::string extractRobotName( const std::string& hw_namespace )
  {
    std::string hw_name = hw_namespace;
    hw_name      .erase(0,1);
    std::replace( hw_name.begin(), hw_name.end(), '/', '_');
    return hw_name;
  }

  /**
   * @brief The RobotHW class
   */
  class RobotHW: public hardware_interface::RobotHW
  {
  public:

    RobotHW( );
    ~RobotHW( );

    // ======================================================= final methods (cannot be overriden by the derived clases
    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) final;
    virtual void read(const ros::Time& time, const ros::Duration& period) final;
    virtual void write(const ros::Time& time, const ros::Duration& period) final;
    virtual bool prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list) final;
    virtual bool checkForConflict(const std::list< hardware_interface::ControllerInfo >& info) final;
    virtual bool shutdown() final;
    // ======================================================= End - final methods



    // ======================================================= Method to override inthe derived classes
    virtual bool doInit               ( ) { return true; }
    virtual bool doShutdown           ( ) { return true; }
    virtual bool doRead               (const ros::Time& time, const ros::Duration& period) { return true; }
    virtual bool doWrite              (const ros::Time& time, const ros::Duration& period) { return true; }
    virtual bool doPrepareSwitch      (const std::list< hardware_interface::ControllerInfo >& start_list
                                      ,const std::list< hardware_interface::ControllerInfo >& stop_list) { return true; }
    virtual bool doCheckForConflict   (const std::list< hardware_interface::ControllerInfo >& info) { return true; }
    // ======================================================= END - Method to override inthe derived classes


    // =======================================================
    void setResourceNames ( const std::vector<std::string>& resource_names ) { m_resource_names = resource_names; }
    // =======================================================

    // ======================================================= utils
    void diagnostics( diagnostic_updater::DiagnosticStatusWrapper &stat );
    const cnr_hardware_interface::StatusHw&  getStatus           ( ) const { return m_status; }
    const std::string&                       getRobotHwNamespace ( ) const { return m_robot_hw_nh.getNamespace(); }
    // ======================================================= END - utils

  protected:

    virtual bool setParamServer(configuration_msgs::SetConfigRequest& req, configuration_msgs::SetConfigResponse& res);
    virtual bool getParamServer(configuration_msgs::GetConfigRequest& req, configuration_msgs::GetConfigResponse& res);

    void add_diagnostic_message(const std::string& level, const std::string& summary, const std::map<std::string, std::string>& key_values, const bool verbose = false );
    virtual bool dump_state( const cnr_hardware_interface::StatusHw& status ) final;
    virtual bool dump_state( ) final;

  private:
    virtual bool enterInit            (ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
    virtual bool enterShutdown        ( );
    virtual bool enterWrite           ( );
    virtual bool enterPrepareSwitch   (const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list);
    virtual bool enterCheckForConflict(const std::list< hardware_interface::ControllerInfo >& info);

    virtual bool exitInit             ( );
    virtual bool exitShutdown         ( );
    virtual bool exitWrite            ( );
    virtual bool exitPrepareSwitch    ( );
    virtual bool exitCheckForConflict ( ) { return true; }


  protected:

    std::string                                      m_robot_name;
    ros::NodeHandle                                  m_root_nh;
    ros::NodeHandle                                  m_robot_hw_nh;
    ros::CallbackQueue                               m_robot_hw_queue;
    std::shared_ptr<cnr_logger::TraceLogger>         m_logger;

    SetParamFcn                                      m_set_param;

    std::mutex                                       m_mutex;
    diagnostic_msgs::DiagnosticArray                 m_diagnostic;
    ros::ServiceServer                               m_get_params_server;
    ros::ServiceServer                               m_set_params_server;
    bool                                             m_stop_thread;

    bool                                             m_is_first_read;
    cnr_hardware_interface::StatusHw                 m_prev_status;
    cnr_hardware_interface::StatusHw                 m_status;
    std::vector<std::string>                         m_status_history;

    std::list< hardware_interface::ControllerInfo >  m_active_controllers;
    std::vector< std::string >                       m_resource_names;
    bool                                             m_shutted_down;


  };

  typedef std::shared_ptr<RobotHW> RobotHWSharedPtr;

}

#endif
