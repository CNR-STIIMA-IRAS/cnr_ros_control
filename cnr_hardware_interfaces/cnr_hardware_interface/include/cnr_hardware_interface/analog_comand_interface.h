#ifndef HARDWARE_INTERFACE_ANALOG_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_ANALOG_COMMAND_INTERFACE_H

#include <cnr_hardware_interface/analog_state_interface.h>

namespace hardware_interface
{
  
  /** \brief A handle used to read and command a single analog. */
  class AnalogHandle : public AnalogStateHandle
  {
  public:
    AnalogHandle() : AnalogStateHandle(), cmd_(0) {}
    
    /**
     * \param js This analog's state handle
     * \param cmd A pointer to the storage for this analog's output command
     */
    AnalogHandle(const AnalogStateHandle& js, double* cmd)
    : AnalogStateHandle(js), cmd_(cmd)
    {
      if (!cmd_)
      {
        throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command data pointer is null.");
      }
    }
    
    void setCommand(double command) {assert(cmd_); *cmd_ = command;}
    double getCommand() const {assert(cmd_); return *cmd_;}
    
  private:
    double* cmd_;
  };
  
  class AnalogCommandInterface : public HardwareResourceManager<AnalogHandle, ClaimResources> {};
  
  /// \ref pwmInterface for commanding pwm-based components.
  class PwmInterface : public AnalogCommandInterface {};
  
  
}

#endif
