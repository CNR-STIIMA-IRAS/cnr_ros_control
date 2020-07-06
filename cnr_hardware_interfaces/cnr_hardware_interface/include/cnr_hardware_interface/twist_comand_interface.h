#ifndef HARDWARE_INTERFACE_TWIST_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_TIST_COMMAND_INTERFACE_H

#include <cnr_hardware_interface/twist_state_interface.h>

namespace hardware_interface
{
  
  /** \brief A handle used to read and command a Twist. */
  class TwistHandle : public TwistStateHandle
  {
  public:
    TwistHandle() : TwistStateHandle(), cmd_(0) {}
    
    /**
     * \param js This Twist's state handle
     * \param cmd A pointer to the storage for this Twist's output command
     */
    TwistHandle(const TwistStateHandle& s, geometry_msgs::Twist* cmd)
    : TwistStateHandle(s), cmd_(cmd)
    {
      if (!cmd_)
      {
        throw HardwareInterfaceException("Cannot create handle '" + s.getName() + "'. Command data pointer is null.");
      }
    }
    
    void setCommand(geometry_msgs::Twist command) {assert(cmd_); *cmd_ = command;}
    geometry_msgs::Twist getCommand() const {assert(cmd_); return *cmd_;}
    
  private:
    geometry_msgs::Twist* cmd_;
  };
  
  class TwistCommandInterface : public HardwareResourceManager<TwistHandle, ClaimResources> {};
    
}

#endif
