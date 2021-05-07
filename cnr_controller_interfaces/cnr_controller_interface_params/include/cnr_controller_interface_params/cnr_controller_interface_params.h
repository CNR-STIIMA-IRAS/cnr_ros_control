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


/**
 * @struct ControllerData
 * @brief The id is the unique id that identify the controller, the 'check_state' field enables/disables the check of 
 * the state in rutime. If disabled, you can load a generic controller that inherited from the standard 
 * controller_interface::Controller<T>
 */
struct ControllerData
{
  std::string id;
  bool check_state;
  //operator const std::string & () const
  //{
  //  return id;
  //}
  ControllerData& operator=(const std::string& rhs)
  {
    id = rhs;
    return *this;
  }
};

std::ostream& operator<<(std::ostream& lhs, const ControllerData& rhs);
bool operator==(const ControllerData& lhs, const ControllerData& rhs);
bool operator==(const std::string& lhs, const ControllerData& rhs);
bool operator==(const ControllerData& lhs, const std::string& rhs);
bool operator!=(const ControllerData& lhs, const ControllerData& rhs);
bool operator!=(const std::string& lhs, const ControllerData& rhs);
bool operator!=(const ControllerData& lhs, const std::string& rhs);

std::vector<std::string> extract_names(const std::vector<ControllerData>& vv);
std::vector<bool> extract_runtime_checks(const std::vector<ControllerData>& vv);

//============ FUNCTIONS TO DEFINE THE PARAMETERS WHERE THE CTRL STATUS IS LOADED
//! extract the list of names from the ControllerState vector
std::vector<std::string> ctrl_get_names(const std::vector<controller_manager_msgs::ControllerState>& controllers);

//! get the full param name with the list of controllers assoicated to the hw 'hw_name'
std::string ctrl_list_param_name(const std::string& hw_name);

//! get the full param name with the status of the controllers mounted on the hw 'hw_name'
std::string ctrl_status_param_name(const std::string& hw_name,const std::string& ctrl_name);

//! get the full param name with the last available status of the controllers mounted on the hw 'hw_name'
std::string ctrl_last_status_param_name(const std::string& hw_name, const std::string& ctrl_name);

//! get the status of the controller
bool ctrl_get_state(const std::string& hw_name, const std::string& ctrl_name, std::string& status, std::string& error,
                      const ros::Duration& watchdog = ros::Duration(0.0));

//! check the status of the controller is the expected one
bool ctrl_check_state(const std::string& hw_name, const std::string& ctrl_name, const std::string& status,
                        std::string& error, const ros::Duration& watchdog = ros::Duration(0.0));

//!
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
  std::string ret = "[";
  for (const auto & w : what) ret += w + ",";
  ret += "]";
  return ret;
}

template< typename T >
inline std::string to_string(const T& what)
{
  std::string ret = "[" + std::to_string(what) + "]";
  return ret;
}

template< >
inline std::string to_string(const std::string& what)
{
  std::string ret = "[" + what + "]";
  return ret;
}


}
}


#endif
