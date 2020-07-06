#ifndef __CNR__NODELET__HARDWARE__INTERFACE__H__
#define __CNR__NODELET__HARDWARE__INTERFACE__H__

#include <nodelet/nodelet.h>
#include <nodelet/loader.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletList.h>

#include <cnr_logger/cnr_logger.h>
#include <cnr_hardware_interface/internal/cnr_robot_hw_utils.h>
#include <cnr_controller_manager_interface/cnr_controller_manager_interface.h>

namespace cnr_hardware_nodelet_interface
{



//============ UTILITIES
template<class MSG>
bool callRequest( ros::ServiceClient& clnt, MSG& msg, std::string& error, const ros::Duration&  watchdog = ros::Duration(0.0) )
{
  if( watchdog.toSec() > 0 )
  {
    if (!clnt.waitForExistence(watchdog))
    {
      error = "The service '"+ clnt.getService() + "' does not exist. Abort." ;
      return false;
    }
  }
  if (!clnt.call(msg))
  {
    error = "The service '" + clnt.getService() + "' is broken. Abort.";
    return false;
  }
  return true;
}


inline
std::string to_string(  const std::vector<std::string>& what, const std::string value_header = "ctrl" )
{
  std::string ret = value_header + ": <";
  for( size_t j=0; j<what.size(); j++  )
  {
    ret += what.at(j) + ( j<what.size()-1 ? ", " : " ");
  }
  ret +=">";
  return ret;
}



class NodeletManagerInterface
{
private:
  ros::NodeHandle                          root_nh_;
  ros::ServiceClient                       load_;
  ros::ServiceClient                       unload_;
  ros::ServiceClient                       list_;
  std::shared_ptr<cnr_logger::TraceLogger> logger_;
  std::string                              error_;

  std::shared_ptr<nodelet::Loader>         nodelet_loader_;


public:
  std::map<std::string, cnr_controller_manager_interface::ControllerManagerInterface> cmi_;

  NodeletManagerInterface(std::shared_ptr<cnr_logger::TraceLogger> log, const std::string& root_ns, const std::string& nodelet_manager_ns = "/configuration_nodelet_manager" );
  ~NodeletManagerInterface( )
  {
    nodelet_loader_.reset();
  }

  ros::NodeHandle& getNamespace       ( );
  std::string      loadServiceName    ( );
  std::string      unloadServiceName  ( );
  std::string      listServiceName    ( );

  bool loadRequest    ( nodelet::NodeletLoad&   msg, std::string& error, const ros::Duration&  watchdog = ros::Duration(0.0) );
  bool unloadRequest  ( nodelet::NodeletUnload& msg, std::string& error, const ros::Duration&  watchdog = ros::Duration(0.0) );
  bool listRequest    ( nodelet::NodeletList&   msg, std::string& error, const ros::Duration&  watchdog = ros::Duration(0.0) );

  const std::string& error() const { return error_; }

  // inline implementation
  bool get_hw_param(ros::NodeHandle &nh, const std::string& hw_name, nodelet::NodeletLoadRequest& request);
  bool list_hw   ( std::vector<std::string>& hw_names_from_nodelet, const ros::Duration& watchdog );
  bool purge_hw  ( const ros::Duration& watchdog );
  bool load_hw   ( const std::string& hw_to_load_name, const ros::Duration& watchdog, bool double_check);
  bool load_hw   ( const std::vector<std::string>& hw_to_load_names, const ros::Duration& watchdog, bool double_check);
  bool unload_hw ( const std::vector<std::string>& hw_to_unload_names, const ros::Duration& watchdog);

};















}
#endif
