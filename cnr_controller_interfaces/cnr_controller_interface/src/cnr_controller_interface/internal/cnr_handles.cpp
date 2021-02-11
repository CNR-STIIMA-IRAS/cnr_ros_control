#include <memory>
#include <thread>
#include <cstring>
#include <string>
#include <sstream>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <rosdyn_utilities/chain_state.h>
#include <cnr_controller_interface/internal/cnr_handles.h>

namespace cnr
{
namespace control
{

HandleIndexes get_index_map(const std::vector<std::string>& names, const rosdyn::Chain& ks)
{
  HandleIndexes ret;
  try
  {
    for(size_t iAx=0; iAx< ks.getActiveJointsName().size(); iAx++ )
    {
      auto it = std::find(names.begin(), names.end(), ks.getJointName(iAx));
      if( it != names.end() )
      {
        ret[ks.getJointName(iAx)] = iAx;
      }
    }
  }
  catch(std::exception& e)
  {
    std::cerr << __FILE__ << ":" << __LINE__ << " " << e.what() << std::endl;
  }
  catch(...)
  {
    std::cerr << __FILE__ << ":" << __LINE__ << " Unhandled exception" << std::endl;
  }
  return ret;
}


}
}

// streaming status of the handle (by reference)
std::ostream& operator<<(std::ostream& os, hardware_interface::JointHandle& rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  os << ", cmd: " << rhs.getCommand();
  return os;
}

std::ostream& operator<<(std::ostream& os, hardware_interface::JointStateHandle& rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  return os;
}

std::ostream& operator<<(std::ostream& os, hardware_interface::VelEffJointHandle& rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  os << ", cmd: qd" << rhs.getCommandVelocity() << ", eff " << rhs.getCommandEffort();
  return os;
}

std::ostream& operator<<(std::ostream& os, hardware_interface::PosVelEffJointHandle& rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ",  qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  os << ", cmd: q"<< rhs.getCommandPosition()<<", qd " << rhs.getCommandVelocity() << ", eff " << rhs.getCommandEffort();
  return os;
}


// streaming status of the handle (by value)
std::ostream& operator<<(std::ostream& os, hardware_interface::JointHandle rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  os << ", cmd: " << rhs.getCommand();
  return os;
}

std::ostream& operator<<(std::ostream& os, hardware_interface::JointStateHandle rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  return os;
}

std::ostream& operator<<(std::ostream& os, hardware_interface::VelEffJointHandle rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  os << ", cmd: qd" << rhs.getCommandVelocity() << ", eff " << rhs.getCommandEffort();
  return os;
}

std::ostream& operator<<(std::ostream& os, hardware_interface::PosVelEffJointHandle rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ",  qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  os << ", cmd: q"<< rhs.getCommandPosition()<<", qd " << rhs.getCommandVelocity() << ", eff " << rhs.getCommandEffort();
  return os;
}

namespace std
{
// streaming status of the handle (by reference)
// streaming status of the handle (by value)
::std::string to_string(const hardware_interface::JointHandle& rhs)
{
  std::stringstream ss;
  ss << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  ss << ", cmd: " << rhs.getCommand();
  return ss.str();
}

::std::string to_string(const hardware_interface::JointStateHandle& rhs)
{
  std::stringstream ss;
  ss << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  return ss.str();
}

::std::string to_string(const hardware_interface::VelEffJointHandle& rhs)
{
  std::stringstream ss;
  ss << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  ss << ", cmd: qd" << rhs.getCommandVelocity() << ", eff " << rhs.getCommandEffort();
  return ss.str();
}

::std::string to_string(const hardware_interface::PosVelEffJointHandle& rhs)
{
  std::stringstream ss;
  ss << rhs.getName() << ", state: q" << rhs.getPosition() << ",  qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  ss << ", cmd: q"<< rhs.getCommandPosition()<<", qd " << rhs.getCommandVelocity() << ", eff " << rhs.getCommandEffort();
  return ss.str();
}
}
