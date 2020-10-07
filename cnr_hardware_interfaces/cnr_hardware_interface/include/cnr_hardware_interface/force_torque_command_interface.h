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
#ifndef CNR_HARDWARE_INTERFACE_FORCE_TORQUE_COMMAND_INTERFACE_H
#define CNR_HARDWARE_INTERFACE_FORCE_TORQUE_COMMAND_INTERFACE_H

#include <string>
#include <cnr_hardware_interface/force_torque_state_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single analog. */
class ForceTorqueHandle : public ForceTorqueStateHandle
{
public:
  ForceTorqueHandle() : output_force_(nullptr), output_torque_(nullptr) {}

  ForceTorqueHandle(const std::string& name, const std::string& frame_id)
    : ForceTorqueStateHandle(name, frame_id), output_force_(nullptr), output_torque_(nullptr) {}

  ForceTorqueHandle(const std::string& name, const std::string& frame_id, double* force, double* torque)
    : ForceTorqueStateHandle(name, frame_id), output_force_(force), output_torque_(torque) {}

  ForceTorqueHandle(const ForceTorqueStateHandle& h, double* output_force, double* output_torque)
    : ForceTorqueStateHandle(h), output_force_(output_force), output_torque_(output_torque)
  {
    if (!output_force_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + h.getName() +
                                        "'. Force Command data pointer is null.");
    }
    if (!output_torque_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + h.getName() +
                                        "'. Torque Command data pointer is null.");
    }
  }

  void setForce(double* force)
  {
    assert(output_force_);
    output_force_[0] = force[0];
    output_force_[1] = force[1];
    output_force_[2] = force[2];
  }
  void setTorque(double* torque)
  {
    assert(output_torque_);
    output_torque_[0] = torque[0];
    output_torque_[1] = torque[1];
    output_torque_[2] = torque[2];
  }
  double getForceOutput() const
  {
    assert(output_force_);
    return *output_force_;
  }
  double getTorqueOutput() const
  {
    assert(output_torque_);
    return *output_torque_;
  }

private:
  double* output_force_;
  double* output_torque_;
};

class ForceTorqueInterface : public HardwareResourceManager<ForceTorqueHandle, ClaimResources> {};

}  // namespace hardware_interface

#endif  // CNR_HARDWARE_INTERFACE_FORCE_TORQUE_COMMAND_INTERFACE_H
