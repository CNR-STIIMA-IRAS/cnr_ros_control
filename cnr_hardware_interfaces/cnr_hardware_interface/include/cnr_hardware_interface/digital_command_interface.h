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

#ifndef  CNR_HARDWARE_INTERFACE_DIGITAL_COMMAND_INTERFACE_H
#define  CNR_HARDWARE_INTERFACE_DIGITAL_COMMAND_INTERFACE_H

#include <cnr_hardware_interface/digital_state_interface.h>

namespace hardware_interface
{

class DigitalHandle : public DigitalStateHandle
{
public:
  DigitalHandle() : DigitalStateHandle(), cmd_(0) {}

  DigitalHandle(const DigitalStateHandle& js, bool* cmd)
    : DigitalStateHandle(js), cmd_(cmd)
  {
    if (!cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command data pointer is null.");
    }
  }

  void setCommand(bool command)
  {
    assert(cmd_);
    *cmd_ = command;
  }
  bool getCommand() const
  {
    assert(cmd_);
    return *cmd_;
  }

private:
  bool* cmd_;
};

class DigitalCommandInterface : public HardwareResourceManager<DigitalHandle, ClaimResources> {};


}  // namespace hardware_interface

#endif  // CNR_HARDWARE_INTERFACE_DIGITAL_COMMAND_INTERFACE_H
