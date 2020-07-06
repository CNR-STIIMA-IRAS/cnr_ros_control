#ifndef HARDWARE_INTERFACE_DIGITAL_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_DIGITAL_COMMAND_INTERFACE_H

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
    
    void setCommand(bool command) {assert(cmd_); *cmd_ = command;}
    bool getCommand() const {assert(cmd_); return *cmd_;}
    
  private:
    bool* cmd_;
  };
  
  class DigitalCommandInterface : public HardwareResourceManager<DigitalHandle, ClaimResources> {};
  
  
}

#endif
