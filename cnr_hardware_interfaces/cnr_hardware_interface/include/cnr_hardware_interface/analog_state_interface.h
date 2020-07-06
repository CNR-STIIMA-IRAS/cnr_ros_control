
#ifndef HARDWARE_INTERFACE_ANALOG_INPUT_INTERFACE_H
#define HARDWARE_INTERFACE_ANALOG_INPUT_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{
  
  /** A handle used to read the state of a single joint. */
  class AnalogStateHandle
  {
  public:
    AnalogStateHandle() : name_(), value_(0) {}
    
    /**
     * \param name The name of the Analog State
     * \param value A pointer to the storage for this analog state
     */
    AnalogStateHandle(const std::string& name, const double* value)
    : name_(name), value_(value)
    {
      if (!value)
      {
        throw HardwareInterfaceException("Cannot create handle '" + name + "'. Analog data pointer is null.");
      }
    }
    
    std::string getName() const {return name_;}
    double getValue()  const {assert(value_); return *value_;}
    
  private:
    std::string name_;
    const double* value_;
  };
  
  /** \brief Hardware interface to support reading the state of an array of joints
   * 
   * This \ref HardwareInterface supports reading the state of an array of named
   * joints, each of which has some valueition, velocity, and effort (force or
   * torque).
   *
   */
  class AnalogStateInterface : public HardwareResourceManager<AnalogStateHandle> {};
  
}
#endif 
