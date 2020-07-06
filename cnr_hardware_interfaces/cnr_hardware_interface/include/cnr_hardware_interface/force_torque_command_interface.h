#ifndef HARDWARE_INTERFACE_FT_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_FT_COMMAND_INTERFACE_H

#include <cnr_hardware_interface/force_torque_state_interface.h>

namespace hardware_interface
{
  
  /** \brief A handle used to read and command a single analog. */
  class ForceTorqueHandle : public ForceTorqueStateHandle
  {
  public:
    ForceTorqueHandle() : output_force_(nullptr) , output_torque_(nullptr) {}
      
    ForceTorqueHandle(const std::string& name, const std::string& frame_id) 
      : ForceTorqueStateHandle(name, frame_id), output_force_(nullptr) , output_torque_(nullptr) {}
    
    ForceTorqueHandle(const std::string& name, const std::string& frame_id, double* force, double* torque)
      : ForceTorqueStateHandle(name, frame_id), output_force_(force), output_torque_( torque ) {}
 
    ForceTorqueHandle(const ForceTorqueStateHandle& h, double* output_force, double* output_torque)
    : ForceTorqueStateHandle( h ), output_force_(output_force), output_torque_(output_torque)
    {
      if (!output_force_)
      {
        throw HardwareInterfaceException("Cannot create handle '" + h.getName() + "'. Force Command data pointer is null.");
      }
      if (!output_torque_)
      {
        throw HardwareInterfaceException("Cannot create handle '" + h.getName() + "'. Torque Command data pointer is null.");
      }
    }
    
    void setForce ( double* force  ) { assert(output_force_  );  output_force_[0] = force[0]; output_force_[1] = force[1];  output_force_[2] = force[2];}
    void setTorque( double* torque ) { assert(output_torque_ ); output_torque_[0] = torque[0]; output_torque_[1] = torque[1];  output_torque_[2] = torque[2];}
    double getForceOutput ( ) const  { assert(output_force_  ); return *output_force_;}
    double getTorqueOutput( ) const  { assert(output_torque_ ); return *output_torque_;}
    
  private:
    double* output_force_;
    double* output_torque_;
  };
  
  class ForceTorqueInterface : public HardwareResourceManager<ForceTorqueHandle, ClaimResources> {};
  
}

#endif
