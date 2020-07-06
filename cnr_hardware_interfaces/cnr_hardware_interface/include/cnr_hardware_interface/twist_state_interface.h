
#ifndef HARDWARE_INTERFACE_TWIST_INPUT_INTERFACE_H
#define HARDWARE_INTERFACE_TWIST_INPUT_INTERFACE_H

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
    
    std::string getName() const { return name_; }
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
  
}
#endif 
