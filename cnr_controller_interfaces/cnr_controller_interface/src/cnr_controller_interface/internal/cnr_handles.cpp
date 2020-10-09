#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>
#include <cnr_controller_interface/internal/cnr_handles.h>

namespace cnr_controller_interface
{
  
const KinematicStatus * const getPtr( const KinematicStatusConstPtr& in)
{
  return in.get();
}

const KinematicStatus * const getPtr( const KinematicStatus& in)
{
  return &in;
}


KinematicStatus* getPtr(KinematicStatusPtr& in)
{
  return in.get();
}

KinematicStatus* getPtr( KinematicStatus& in)
{
  return &in;
}


HandleIndexes get_index_map(const std::vector<std::string>& names, cnr_controller_interface::KinematicsStructConstPtr ks)
{ 
  HandleIndexes ret;
  try
  { 
    for(size_t iAx=0; iAx< ks->jointNames().size(); iAx++ )
    {
      auto it = std::find(names.begin(), names.end(), ks->jointName(iAx));
      if( it != names.end() )
      {
        ret[ks->jointName(iAx)] = iAx;
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



HandleIndexes get_index_map(const std::vector<std::string>& names, cnr_controller_interface::KinematicStatusConstPtr ks)
{ 
  HandleIndexes ret;
  try
  { 
    for(size_t iAx=0; iAx< ks->jointNames().size(); iAx++ )
    {
      auto it = std::find(names.begin(), names.end(), ks->jointName(iAx));
      if( it != names.end() )
      {
        ret[ks->jointName(iAx)] = iAx;
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