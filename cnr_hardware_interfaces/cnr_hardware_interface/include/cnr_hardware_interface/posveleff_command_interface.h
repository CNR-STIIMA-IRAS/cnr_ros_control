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

#ifndef CNR_HARDWARE_INTERFACE_POSVELEFF_COMMAND_INTERFACE_H
#define CNR_HARDWARE_INTERFACE_POSVELEFF_COMMAND_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cnr_hardware_interface/veleff_command_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class PosVelEffJointHandle : public VelEffJointHandle
{
public:
  PosVelEffJointHandle() : VelEffJointHandle(), cmd_pos_(0) {}

  /**
   * \param js This joint's state handle
   * \param cmd_pos A pointer to the storage for this joint's output command position
   * \param cmd_vel A pointer to the storage for this joint's output command velocity
   * \param eff_cmd A pointer to the storage for this joint's output command acceleration
   */
  PosVelEffJointHandle(const JointStateHandle& js, double* cmd_pos, double* cmd_vel, double* cmd_eff)
    : VelEffJointHandle(js, cmd_vel, cmd_eff), cmd_pos_(cmd_pos)
  {
    if (!cmd_pos)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName()
                                        + "'. Command position data pointer is null.");
    }
  }

  void setCommand(double cmd_pos, double cmd_vel, double cmd_eff)
  {
    setCommandPosition(cmd_pos);
    setCommandVelocity(cmd_vel);
    setCommandEffort(cmd_eff);
  }

  void setCommandPosition(double cmd_pos)
  {
    assert(cmd_pos_);
    *cmd_pos_ = cmd_pos;
  }
  double getCommandPosition() const
  {
    assert(cmd_pos_);
    return *cmd_pos_;
  }

private:
  double* cmd_pos_;
};

class PosVelEffJointInterface : public HardwareResourceManager<PosVelEffJointHandle, ClaimResources> {};

}  // namespace hardware_interface

#endif  // CNR_HARDWARE_INTERFACE_POSVELEFF_COMMAND_INTERFACE_H
