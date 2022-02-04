#include <ros/ros.h>
#include <cnr_controller_interface_params/cnr_controller_interface_params.h>


namespace cnr
{
namespace control
{


std::ostream& operator<<(std::ostream& lhs, const ControllerData& rhs)
{
  lhs << rhs.id;
  return lhs;
}

bool operator==(const ControllerData& lhs, const ControllerData& rhs)
{
  return lhs.id == rhs.id;
}

bool operator==(const std::string& lhs, const ControllerData& rhs)
{
  return lhs == rhs.id;
}

bool operator==(const ControllerData& lhs, const std::string& rhs)
{
  return lhs.id == rhs;
}

bool operator!=(const ControllerData& lhs, const ControllerData& rhs)
{
  return lhs.id != rhs.id;
}

bool operator!=(const std::string& lhs, const ControllerData& rhs)
{
  return lhs != rhs.id;
}

bool operator!=(const ControllerData& lhs, const std::string& rhs)
{
  return lhs.id == rhs;
}

std::vector<std::string> extract_names(const std::vector<ControllerData>& vv)
{
  std::vector<std::string> ret;
  for(auto const  & v : vv) ret.push_back(v.id);
  return ret;
}

std::vector<bool> extract_runtime_checks(const std::vector<ControllerData>& vv)
{
  std::vector<bool> ret;
  for(auto const & v : vv) ret.push_back(v.check_state);
  return ret;
}

std::vector<std::string> ctrl_get_names(const std::vector< controller_manager_msgs::ControllerState >& controllers)
{
  std::vector<std::string> ret;
  for (auto & ctrl : controllers) ret.push_back(ctrl.name);
  return ret;
}
//============


//============ FUNCTIONS TO DEFINE THE PARAMETERS WHERE THE CTRL STATUS IS LOADED
std::string ctrl_list_param_name(const std::string& hw_name)
{
  return "/" + hw_name + "/status/controllers_list";
}
std::string ctrl_last_status_param_name(const std::string& hw_name, const std::string& ctrl_name)
{
  return "/" + hw_name + "/status/controllers/" + ctrl_name + "/last_status";
}

// std::string ctrl_status_param_name(const std::string& hw_name, const std::string& ctrl_name)
// {
//   return "/" + hw_name + "/status/controllers/" + ctrl_name + "/status";
// }

// bool ctrl_get_state(const std::string& hw_name, const std::string& ctrl_name, std::string& status, std::string& error,
//                       const ros::Duration& watchdog)
// {
//   ros::Time st = ros::Time::now();
//   const std::string p = ctrl_last_status_param_name(hw_name, ctrl_name);
//   while (ros::ok())
//   {
//     if (ros::param::get(p, status))
//     {
//       break;
//     }

//     if((watchdog.toSec() > 0) && (ros::Time::now() > (st + watchdog)))
//     {
//       error += "Timeout. Param " + p + " does not exist";
//       return false;
//     }
//     ros::Duration(0.001).sleep();
//   }
//   return true;
// }

// bool ctrl_check_state(const std::string& hw_name, const std::string& ctrl_name, const std::string& status,
//                         std::string& error, const ros::Duration& watchdog)
// {
//   bool ok = false;
//   ros::Time st = ros::Time::now();
//   std::string actual_status;
//   const std::string p = ctrl_last_status_param_name(hw_name, ctrl_name);
//   ros::Time n = ros::Time::now();
//   while (ros::ok())
//   {
//     if (ros::param::get(p, actual_status))
//     {
//       if( actual_status == status)
//       {
//         ok = true;
//         break;
//       }
//     }

//     if((watchdog.toSec() > 0) && (ros::Time::now() > (st + watchdog)))
//     {
//       error += "Timeout.";
//       ok = false;
//       break;
//     }
//     ros::Duration(0.001).sleep();
//   }
//   ros::Time a = ros::Time::now();
//   if(!ok)
//   {
//     error += "Failed while checking '" + hw_name + "/" + ctrl_name + "' state. ";
//     error += "The state is " + actual_status + " while '" + status + "' is expected ";
//     error +="(transition waited for " + std::to_string((a-n).toSec()) + "s, ";
//     error +="watchdog: "+ std::to_string(watchdog.toSec())+ "s)";
//     return false;
//   }
//   return true;
// }
//============



}
}
