/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CNR_HARDWARE_INTERFACE_VELEFF_COMMAND_INTERFACE_H
#define CNR_HARDWARE_INTERFACE_VELEFF_COMMAND_INTERFACE_H

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
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                        "'. Command velocity pointer is null.");
    }
    if (!cmd_eff)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                        "'. Command effort  pointer is null.");
    }
  }

  void setCommand(double cmd_vel, double cmd_eff)
  {
    setCommandVelocity(cmd_vel);
    setCommandEffort(cmd_eff);
  }

  void setCommandVelocity(double cmd_vel)
  {
    assert(cmd_vel_);
    *cmd_vel_ = cmd_vel;
  }
  void setCommandEffort(double cmd_eff)
  {
    assert(cmd_eff_);
    *cmd_eff_ = cmd_eff;
  }

  double getCommandVelocity()     const
  {
    assert(cmd_vel_);
    return *cmd_vel_;
  }
  double getCommandEffort()     const
  {
    assert(cmd_eff_);
    return *cmd_eff_;
  }

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

}  // namespace hardware_interface

#endif  // CNR_HARDWARE_INTERFACE_VELEFF_COMMAND_INTERFACE_H
