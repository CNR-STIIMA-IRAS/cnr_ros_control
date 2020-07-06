#ifndef HARDWARE_INTERFACE_POSE_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_POSE_COMMAND_INTERFACE_H

#include <cnr_hardware_interface/pose_state_interface.h>

namespace hardware_interface
{
  
  /** \brief A handle used to read and command a pose. */
  class PoseHandle : public PoseStateHandle
  {
  public:
    PoseHandle() : PoseStateHandle(), cmd_(0) {}
    
    /**
     * \param js This Pose's state handle
     * \param cmd A pointer to the storage for this Pose's output command
     */
    PoseHandle(const PoseStateHandle& s, geometry_msgs::Pose* cmd)
    : PoseStateHandle(s), cmd_(cmd)
    {
      if (!cmd_)
      {
        throw HardwareInterfaceException("Cannot create handle '" + s.getName() + "'. Command data pointer is null.");
      }
    }
    
    void setCommand(geometry_msgs::Pose command) {assert(cmd_); *cmd_ = command;}
    geometry_msgs::Pose getCommand() const {assert(cmd_); return *cmd_;}
    
  private:
    geometry_msgs::Pose* cmd_;
  };
  
  class PoseCommandInterface : public HardwareResourceManager<PoseHandle, ClaimResources> {};
    
}

#endif
