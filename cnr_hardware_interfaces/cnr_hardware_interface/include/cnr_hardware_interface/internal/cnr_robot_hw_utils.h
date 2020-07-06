#ifndef __CNR__ROBOT__HW__UTILS__H__
#define __CNR__ROBOT__HW__UTILS__H__

#include <ros/ros.h>
#include <cnr_hardware_interface/internal/cnr_robot_hw_status.h>

namespace cnr_hardware_interface
{

// Iterator over enumr
typedef cnr_hardware_interface::EnumIterator<cnr_hardware_interface::tagStatusHw,cnr_hardware_interface::tagStatusHw::UNLOADED, cnr_hardware_interface::tagStatusHw::SRV_ERROR> StatusHwIterator;

inline std::string last_status_param( const std::string hw_name ) { return "/" + hw_name + "/status/last_status"; }
inline std::string status_param     ( const std::string hw_name ) { return "/" + hw_name + "/status/status"; }

inline
bool get_state(ros::NodeHandle& nh, const std::string& hw_name, cnr_hardware_interface::StatusHw& status, const ros::Duration& watchdog, std::string& error )
{
  std::string state;
  ros::Time st = ros::Time::now();
  error += " GET_STATE: nh: " + nh.getNamespace() + " param: " + last_status_param( hw_name );
  bool ok = false;
  do
  {
    if( nh.getParam( last_status_param( hw_name ), state ))
    {
      for( const StatusHw& it : StatusHwIterator() )
      {
        if( state == to_string( it ) ) { status = it; ok = true; break; }
      }
    }


    if( ( watchdog.toSec() > 0 ) && ( ros::Time::now() - st > watchdog ) )
    {
      error += " Timeout";
      break;
    }
  } while( ros::ok() && !ok );

  return ok;
}

inline
bool set_state(ros::NodeHandle& nh, const std::string& hw_name, const cnr_hardware_interface::StatusHw& status )
{

  nh.setParam( status_param( hw_name ), cnr_hardware_interface::to_string( status ) );
  return true;
}


inline
bool get_state(ros::NodeHandle& nh, const std::vector<std::string> &hw_names, std::vector<cnr_hardware_interface::StatusHw>& status, std::string& error, const ros::Duration& watchdog)
{
  status.clear();
  for( auto const & hw_name : hw_names )
  {
    cnr_hardware_interface::StatusHw st;
    if( !get_state( nh, hw_name, st, watchdog, error ) )
    {
      error = "RobotHW (" + hw_name + ") Error Status" ;
      return false;
    }
    status.push_back( st );
  }
  return true;
}


}


#endif
