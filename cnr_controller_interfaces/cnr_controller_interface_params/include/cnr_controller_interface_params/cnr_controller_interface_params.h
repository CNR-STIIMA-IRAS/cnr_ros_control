#ifndef CNR_CONTROLLER_INTERFACE_PARAMS__CNR_CONTROLLER_INTERFACE_PARAMS__H
#define CNR_CONTROLLER_INTERFACE_PARAMS__CNR_CONTROLLER_INTERFACE_PARAMS__H

#include <vector>
#include <string>
#include <ros/duration.h>
#include <controller_manager_msgs/ControllerState.h>

namespace cnr
{
namespace control
{


//============ FUNCTIONS TO DEFINE THE PARAMETERS WHERE THE CTRL STATUS IS LOADED
std::vector<std::string> get_names(const std::vector<controller_manager_msgs::ControllerState>& controllers);
std::string              ctrl_list_param(const std::string& hw_name);
std::string              last_status_param(const std::string& hw_name,
                                           const std::string& ctrl_name);
std::string              status_param(const std::string& hw_name,
                                      const std::string& ctrl_name);
bool                     get_state(const std::string& hw_name,
                                   const std::string& ctrl_name,
                                   std::string& status,
                                   std::string& error,
                                   const ros::Duration& watchdog = ros::Duration(0.0));

bool                     check_state(const std::string& hw_name,
                                     const std::string& ctrl_name,
                                     const std::string& status,
                                     std::string& error,
                                     const ros::Duration& watchdog = ros::Duration(0.0));


template< typename T >
inline std::string to_string(const std::vector< T >& what)
{
  std::string ret = "< ";
  for (const auto & w : what) ret += std::to_string(w) + " ";
  ret += " >";
  return ret;
}

template<>
inline std::string to_string(const std::vector<std::string>& what)
{
  std::string ret = "< ";
  for (const auto & w : what) ret += w + " ";
  ret += " >";
  return ret;
}

template< typename T >
inline std::string to_string(const T& what)
{
  std::string ret = "< " + std::to_string(what) + " >";
  return ret;
}

template< >
inline std::string to_string(const std::string& what)
{
  std::string ret = "< " + what + " >";
  return ret;
}


}
}


#endif
