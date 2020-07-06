#ifndef __CNR__CONTROLLER_INTERFACE__UTILS__
#define __CNR__CONTROLLER_INTERFACE__UTILS__

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_interface/controller.h>
#include <controller_interface/controller_base.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <cnr_logger/cnr_logger.h>

#include <cnr_controller_manager_interface/internal/utils.h>

namespace cnr_controller_manager_interface
{




//============ CLASS TO CONFIGURE THE SERVICES TO MANAGE THE CTRLS
class ControllerManagerInterface
{
private:
  ros::NodeHandle                          nh_;
  ros::ServiceClient                       load_;
  ros::ServiceClient                       unload_;
  ros::ServiceClient                       doswitch_;
  ros::ServiceClient                       list_;
  ros::ServiceClient                       list_types_;
  std::shared_ptr<cnr_logger::TraceLogger> logger_;


  std::string                              error_;
public:
  ControllerManagerInterface( std::shared_ptr<cnr_logger::TraceLogger> log, const std::string& hw_name ) : nh_( "/" + hw_name ), logger_(log)
  {
    list_       = nh_.serviceClient<controller_manager_msgs::ListControllers>     ("/"+ hw_name +"/controller_manager/list_controllers");
    list_types_ = nh_.serviceClient<controller_manager_msgs::ListControllerTypes> ("/"+ hw_name +"/controller_manager/list_controller_types");
    load_       = nh_.serviceClient<controller_manager_msgs::LoadController>      ("/"+ hw_name +"/controller_manager/load_controller");
    unload_     = nh_.serviceClient<controller_manager_msgs::UnloadController>    ("/"+ hw_name +"/controller_manager/unload_controller");
    doswitch_   = nh_.serviceClient<controller_manager_msgs::SwitchController>    ("/"+ hw_name +"/controller_manager/switch_controller");
    assert( logger_ );
  }

  std::string  getHwName          ( ) { std::string n = nh_.getNamespace(); n.erase(0,1); return n;}
  std::string  getNamespace       ( ) { return nh_.getNamespace(); }
  std::string  listServiceName    ( ) { return list_.getService(); }
  std::string  listTypeServiceName( ) { return list_types_.getService(); }
  std::string  loadServiceName    ( ) { return load_.getService(); }
  std::string  unloadServiceName  ( ) { return unload_.getService(); }
  std::string  doswitchServiceName( ) { return doswitch_.getService(); }

  bool loadRequest    ( controller_manager_msgs::LoadController&      msg, std::string& error, const ros::Duration&  watchdog = ros::Duration(0.0) ) { return callRequest(load_      , msg, error, watchdog); }
  bool unloadRequest  ( controller_manager_msgs::UnloadController&    msg, std::string& error, const ros::Duration&  watchdog = ros::Duration(0.0) ) { return callRequest(unload_    , msg, error, watchdog); }
  bool switchRequest  ( controller_manager_msgs::SwitchController&    msg, std::string& error, const ros::Duration&  watchdog = ros::Duration(0.0) ) { return callRequest(doswitch_  , msg, error, watchdog); }
  bool listRequest    ( controller_manager_msgs::ListControllers&     msg, std::string& error, const ros::Duration&  watchdog = ros::Duration(0.0) ) { return callRequest(list_      , msg, error, watchdog); }
  bool listTypeRequest( controller_manager_msgs::ListControllerTypes& msg, std::string& error, const ros::Duration&  watchdog = ros::Duration(0.0) ) { return callRequest(list_types_, msg, error, watchdog); }

  std::shared_ptr<cnr_logger::TraceLogger> getLogger() { return logger_; }
  const std::string& error() const { return error_; }

  bool list_ctrl  ( std::vector< controller_manager_msgs::ControllerState >&  running
                  , std::vector< controller_manager_msgs::ControllerState >&  stopped
                  , const ros::Duration&                                      watchdog = ros::Duration(0.0));

  bool load_ctrl ( const std::string&              to_load_name , const ros::Duration& watchdog );
  bool load_ctrl ( const std::vector<std::string>& to_load_names, const ros::Duration& watchdog );

  bool doswitch( const int                        strictness
               , const std::vector<std::string>*  to_load_and_start_names
               , const std::vector<std::string>*  to_restart_names
               , const std::vector<std::string>*  to_stop_unload_names
               , const ros::Duration&             watchdog = ros::Duration(0.0));

  bool doswitch( const int&                                                    strictness
               , const std::vector<controller_manager_msgs::ControllerState>*  to_load_and_start_names
               , const std::vector<controller_manager_msgs::ControllerState>*  to_restart_names
               , const std::vector<controller_manager_msgs::ControllerState>*  to_stop_unload_names
               , const ros::Duration&                                          watchdog = ros::Duration(0.0) );

  bool doswitch( const int                              strictness
               , const std::vector<std::string> * const next_ctrl
               , const ros::Duration&                   watchdog = ros::Duration(0.0)   );

  bool unload_ctrl( const std::string&                                           to_unload_name , const ros::Duration& watchdog = ros::Duration(0.0));
  bool unload_ctrl( const std::vector<std::string>&                              to_unload_names, const ros::Duration& watchdog = ros::Duration(0.0));
  bool unload_ctrl( const std::vector<controller_manager_msgs::ControllerState>& to_unload_names, const ros::Duration& watchdog = ros::Duration(0.0));

  bool match_ctrl ( const std::vector<std::string>& names, const ros::Duration&  watchdog = ros::Duration(0.0));


  bool stop_unload_ctrl     (const std::vector<std::string>&  ctrl_to_stop_unload_names, const ros::Duration& watchdog = ros::Duration(0.0));
  bool stop_unload_all_ctrl (const ros::Duration& watchdog = ros::Duration(0.0));



};
//============


bool stop_unload_all_ctrl (std::map<std::string, ControllerManagerInterface> &ctrl_srvs, const std::vector<std::string>& hw_to_unload_names, const ros::Duration& watchdog);








//




;

}

#endif
