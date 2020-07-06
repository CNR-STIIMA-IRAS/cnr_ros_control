
#ifndef HARDWARE_INTERFACE_VELEFF_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_VELEFF_COMMAND_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class VelEffJointHandle : public JointStateHandle
{
public:
  VelEffJointHandle() : JointStateHandle(), cmd_vel_(0), cmd_eff_(0) {}

  /**
   * \param js This joint's state handle
   * \param cmd_vel A pointer to the storage for this joint's output command velocity
   * \param cmd_eff A pointer to the storage for this joint's output command effort
   */
  VelEffJointHandle(const JointStateHandle& js, double* cmd_vel, double* cmd_eff)
    : JointStateHandle(js), cmd_vel_(cmd_vel), cmd_eff_(cmd_eff)
  {
    if (!cmd_vel)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command velocity pointer is null.");
    }
    if (!cmd_eff)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command effort  pointer is null.");
    }
  }

  void setCommand(double cmd_vel, double cmd_eff)
  {
    setCommandVelocity(cmd_vel);
    setCommandEffort(cmd_eff);
  }

  void setCommandVelocity(double cmd_vel)     {assert(cmd_vel_); *cmd_vel_ = cmd_vel;}
  void setCommandEffort(double cmd_eff)     {assert(cmd_eff_); *cmd_eff_ = cmd_eff;}

  double getCommandVelocity()     const {assert(cmd_vel_); return *cmd_vel_;}
  double getCommandEffort()     const {assert(cmd_eff_); return *cmd_eff_;}

private:
  double* cmd_vel_;
  double* cmd_eff_;
};

/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding joints by velocity, effort
 * together in one command.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class VelEffJointInterface : public HardwareResourceManager<VelEffJointHandle, ClaimResources> {};

}

#endif /*HARDWARE_INTERFACE_POSVEL_COMMAND_INTERFACE_H*/
