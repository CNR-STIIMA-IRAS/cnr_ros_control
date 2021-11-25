#include <cnr_controller_interface_utils/cnr_controller_interface_utils.h>


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

}
}
