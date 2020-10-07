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

#ifndef CNR_HARDWARE_INTERFACE_FORCE_TORQUE_STATE_INTERFACE_H
#define CNR_HARDWARE_INTERFACE_FORCE_TORQUE_STATE_INTERFACE_H

#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{

/// A handle used to read the state of a force-torque sensor.
class ForceTorqueStateHandle
{
public:
  ForceTorqueStateHandle() : name_(""), frame_id_(""), force_(0), torque_(0) {}

  explicit ForceTorqueStateHandle(const std::string& name, const std::string& frame_id)
  : name_(name), frame_id_(frame_id), force_(0), torque_(0)
  {
    // nothing to do so far
  }

  /**
   * \param name The name of the sensor
   * \param frame_id The reference frame to which this sensor is associated
   * \param force A pointer to the storage of the force value: a triplet (x,y,z)
   * \param torque A pointer to the storage of the torque value: a triplet (x,y,z)
   *
   */
  ForceTorqueStateHandle(const std::string& name,
                         const std::string& frame_id,
                         const double* force,
                         const double* torque)
    : name_(name),
      frame_id_(frame_id),
      force_(force),
      torque_(torque)
  {}

  std::string getName()     const
  {
    return name_;
  }
  std::string getFrameId()  const
  {
    return frame_id_;
  }
  const double* getForce()  const
  {
    return force_;
  }
  const double* getTorque() const
  {
    return torque_;
  }

private:
  std::string name_;
  std::string frame_id_;
  const double* force_;
  const double* torque_;
};

/** \brief Hardware interface to support reading the state of a force-torque sensor. */
class ForceTorqueStateInterface : public HardwareResourceManager<ForceTorqueStateHandle> {};

}  // namespace hardware_interface

#endif  // CNR_HARDWARE_INTERFACE_FORCE_TORQUE_STATE_INTERFACE_H
