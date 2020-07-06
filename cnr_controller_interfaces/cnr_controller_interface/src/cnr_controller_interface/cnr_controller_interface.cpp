#include <cnr_controller_interface/cnr_controller_interface.h>

namespace cnr_controller_interface
{



std::vector<std::string> get_names( const std::vector< controller_manager_msgs::ControllerState >& controllers )

{
  std::vector<std::string> ret;
  for( auto & ctrl : controllers) ret.push_back(ctrl.name);
  return ret;
}
//============







//============ FUNCTIONS TO DEFINE THE PARAMETERS WHERE THE CTRL STATUS IS LOADED
std::string ctrl_list_param  ( const std::string& hw_name )
{
  return "/" + hw_name + "/status/controllers_list";
}
std::string last_status_param( const std::string& hw_name, const std::string& ctrl_name )
{
  return "/" + hw_name + "/status/controllers/" + ctrl_name + "/last_status";
}

std::string status_param     ( const std::string& hw_name, const std::string& ctrl_name )
{
  return "/" + hw_name + "/status/controllers/" + ctrl_name + "/status";
}
bool get_state( const std::string& hw_name, const std::string& ctrl_name, std::string& status, std::string& error, const ros::Duration& watchdog )
{
  ros::Time st = ros::Time::now();
  const std::string p = last_status_param(hw_name,ctrl_name);
  while( ros::ok() )
  {
    if( ros::param::get( p , status ) )
    {
      break;
    }

    if( (watchdog.toSec() > 0 ) && (ros::Time::now() > (st + watchdog ) ) )
    {
      error += "Timeout. Param " + p + " does not exist";
      return false;
    }
    ros::Duration(0.001).sleep();
  }
  return true;
}
//============





}
