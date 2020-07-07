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

#ifndef CNR_HARDWARE_INTERFACE_TWIST_STATE_INTERFACE_H
#define CNR_HARDWARE_INTERFACE_TWIST_STATE_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <geometry_msgs/Twist.h>

namespace hardware_interface
{

/** A handle used to read the state of a single joint. */
class TwistStateHandle
{
public:
  TwistStateHandle() : name_(), value_(0) {}

  /**
   * \param name The name of the Twist State
   * \param value A pointer to the storage for this Twist state
   */
  TwistStateHandle(const std::string& name, const geometry_msgs::Twist* value)
    : name_(name), value_(value)
  {
    if (!value)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Twist data pointer is null.");
    }
  }

  std::string getName() const
  {
    return name_;
  }
  const geometry_msgs::Twist* getFrame()  const
  {
    assert(value_);
    return value_;
  }


private:
  std::string name_;
  const geometry_msgs::Twist* value_;
};

/** \brief Hardware interface to support reading the state of an array of joints
 *
 * This \ref HardwareInterface supports reading the state of an array of named
 * joints, each of which has some valueition, velocity, and effort (force or
 * torque).
 *
 */
class TwistStateInterface : public HardwareResourceManager<TwistStateHandle> {};

}  // namespace hardware_interface

#endif  // CNR_HARDWARE_INTERFACE_TWIST_STATE_INTERFACE_H
