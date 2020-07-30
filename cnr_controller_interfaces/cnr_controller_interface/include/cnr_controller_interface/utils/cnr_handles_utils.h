#ifndef CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H
#define CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H

#include <cnr_controller_interface/utils/cnr_kinematic_status.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>

namespace cnr_controller_interface
{

template<>
extract( const hardware_interface::JointStateInterface* in, KinematicStatus& out )
{
  const std::vector<std::string> names = in->getNames();
  if( names.size() == out.nrows() )
  {
    for (size_t iAx = 0; iAx<names.size(); iAx++)
    {
      out.q(iAx) = in->getHandle(in->getNames().at(iAx)).getPosition();
      out.qd(iAx) = in->getHandle(in->getNames().at(iAx)).getVelocity();
      out.qdd(iAx) = 0.0;
      out.effort(iAx) = in->getHandle(in->getNames().at(iAx)).getEffort();
    }
    return true;
  }
  return false;
}

/**
 *hardware_interface::JointCommandInterface
 */
template<>
extract( const hardware_interface::JointCommandInterface* in, KinematicStatus& out )
{
  const std::vector<std::string> names = in->getNames();
  if( names.size() == out.nrows() )
  {
    for (size_t iAx = 0; iAx<names.size(); iAx++)
    {
      out.q(iAx) = in->getHandle(in->getNames().at(iAx)).getPosition();
      out.qd(iAx) = in->getHandle(in->getNames().at(iAx)).getVelocity();
      out.qdd(iAx) = 0.0;
      out.effort(iAx) = in->getHandle(in->getNames().at(iAx)).getEffort();
    }
    return true;
  }
  return false;
}

template<>
extract( const KinematicStatus& in, hardware_interface::JointCommandInterface* out )
{
  const std::vector<std::string> names = out->getNames();
  if( names.size() == in.nrows() )
  {
    for (size_t iAx = 0; iAx<names.size(); iAx++)
    {
      out->getHandle(out->getNames().at(iAx)).setCommand( in.q(iAx) );
    }
    return true;
  }
  return false;
}

/**
 *
 */
template<>
extract( const hardware_interface::VelEffJointInterface* in, KinematicStatus& out )
{
  const std::vector<std::string> names = in->getNames();
  if( names.size() == out.nrows() )
  {
    for (size_t iAx = 0; iAx<names.size(); iAx++)
    {
      out.q(iAx) = in->getHandle(in->getNames().at(iAx)).getPosition();
      out.qd(iAx) = in->getHandle(in->getNames().at(iAx)).getVelocity();
      out.qdd(iAx) = 0.0;
      out.effort(iAx) = in->getHandle(in->getNames().at(iAx)).getEffort();
    }
    return true;
  }
  return false;
}

template<>
extract( const KinematicStatus& in, hardware_interface::VelEffJointInterface* out )
{
  const std::vector<std::string> names = out->getNames();
  if( names.size() == in.nrows() )
  {
    for (size_t iAx = 0; iAx<names.size(); iAx++)
    {
      out->getHandle(out->getNames().at(iAx)).setCommandVelocity(in.qd(iAx) );
      out->getHandle(out->getNames().at(iAx)).setCommandEffort(in.effort(iAx) );
    }
    return true;
  }
  return false;
}

/**
 *
 */
template<>
extract( const hardware_interface::PosVelEffJointInterface* in, KinematicStatus& out )
{
  const std::vector<std::string> names = in->getNames();
  if( names.size() == out.nrows() )
  {
    for (size_t iAx = 0; iAx<names.size(); iAx++)
    {
      out.q(iAx) = in->getHandle(in->getNames().at(iAx)).getPosition();
      out.qd(iAx) = in->getHandle(in->getNames().at(iAx)).getVelocity();
      out.qdd(iAx) = 0.0;
      out.effort(iAx) = in->getHandle(in->getNames().at(iAx)).getEffort();
    }
    return true;
  }
  return false;
}

template<>
extract( const KinematicStatus& in, hardware_interface::PosVelEffJointInterface* out )
{
  const std::vector<std::string> names = out->getNames();
  if( names.size() == in.nrows() )
  {
    for (size_t iAx = 0; iAx<names.size(); iAx++)
    {
      out->getHandle(out->getNames().at(iAx)).setCommandPosition(in.qd(iAx) );
      out->getHandle(out->getNames().at(iAx)).setCommandVelocity(in.qd(iAx) );
      out->getHandle(out->getNames().at(iAx)).setCommandEffort(in.effort(iAx) );
    }
    return true;
  }
  return false;
}
}
#endif  // CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H
