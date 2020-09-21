#ifndef CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H
#define CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H

#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>

namespace cnr_controller_interface
{

/**
 * JointStateInterface
 */

bool get_from_hw(hardware_interface::JointStateInterface* hi, cnr_controller_interface::KinematicStatus& ks);
bool get_from_hw(hardware_interface::JointStateInterface* hi, KinematicStatusPtr&                        ks);

/**
 * hardware_interface::VelEffJointInterface
 */
bool get_from_hw(hardware_interface::VelEffJointInterface*   hi, cnr_controller_interface::KinematicStatus&    ks);
bool get_from_hw(hardware_interface::VelEffJointInterface*   hi, cnr_controller_interface::KinematicStatusPtr& ks);
bool set_to_hw(cnr_controller_interface::KinematicStatus&    ks, hardware_interface::VelEffJointInterface*     hi);
bool set_to_hw(cnr_controller_interface::KinematicStatusPtr& ks, hardware_interface::VelEffJointInterface*     hi);

/**
 * PosVelEffJointInterface
 */
bool get_from_hw(hardware_interface::PosVelEffJointInterface* hi, cnr_controller_interface::KinematicStatus&    ks);
bool get_from_hw(hardware_interface::PosVelEffJointInterface* hi, cnr_controller_interface::KinematicStatusPtr& ks);
bool set_to_hw(cnr_controller_interface::KinematicStatus&     ks, hardware_interface::PosVelEffJointInterface*  hi);
bool set_to_hw(cnr_controller_interface::KinematicStatusPtr&  ks, hardware_interface::PosVelEffJointInterface*  hi);

/**
 * JointCommandInterface
 */

bool get_from_hw(hardware_interface::JointCommandInterface*  in, cnr_controller_interface::KinematicStatus&    ks);
bool get_from_hw(hardware_interface::JointCommandInterface*  in, cnr_controller_interface::KinematicStatusPtr& ks);
bool set_to_hw(cnr_controller_interface::KinematicStatus&    ks, hardware_interface::JointCommandInterface*    hi);
bool set_to_hw(cnr_controller_interface::KinematicStatusPtr& ks,hardware_interface::JointCommandInterface*     hi);

/**
 * @brief EffortJointInterface
 */
bool get_from_hw(hardware_interface::EffortJointInterface* hi, KinematicStatus&                          ks);
bool get_from_hw(hardware_interface::EffortJointInterface* hi, KinematicStatusPtr&                       ks);
bool set_to_hw(KinematicStatus&                            ks, hardware_interface::EffortJointInterface* hi);
bool set_to_hw(KinematicStatusPtr&                         ks, hardware_interface::EffortJointInterface* hi);

/**
 * @brief VelocityJointInterface
 */
bool get_from_hw(hardware_interface::VelocityJointInterface* hi, KinematicStatus&                            ks);
bool get_from_hw(hardware_interface::VelocityJointInterface* hi, KinematicStatusPtr&                         ks);
bool set_to_hw  (KinematicStatus&                            ks, hardware_interface::VelocityJointInterface* hi);
bool set_to_hw  (KinematicStatusPtr&                         ks, hardware_interface::VelocityJointInterface* hi);

/**
 * @brief VelocityJointInterface
 */

bool get_from_hw(hardware_interface::PositionJointInterface* hi,KinematicStatus&                            ks);
bool get_from_hw(hardware_interface::PositionJointInterface* hi,KinematicStatusPtr&                         ks);
bool set_to_hw  (KinematicStatus&                            ks,hardware_interface::PositionJointInterface* hi);
bool set_to_hw  (KinematicStatusPtr&                         ks,hardware_interface::PositionJointInterface* hi);

}  // namespace hardware_interface




// streaming status of the handle (by reference)
inline std::ostream& operator<<(std::ostream& os, hardware_interface::JointHandle& rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  os << ", cmd: q" << rhs.getCommand();
  return os;
}

inline std::ostream& operator<<(std::ostream& os, hardware_interface::JointStateHandle& rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  return os;
}

inline std::ostream& operator<<(std::ostream& os, hardware_interface::VelEffJointHandle& rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  os << ", cmd: qd" << rhs.getCommandVelocity() << ", eff " << rhs.getCommandEffort();
  return os;
}

inline std::ostream& operator<<(std::ostream& os, hardware_interface::PosVelEffJointHandle& rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ",  qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  os << ", cmd: q"<< rhs.getCommandPosition()<<", qd " << rhs.getCommandVelocity() << ", eff " << rhs.getCommandEffort();
  return os;
}


// streaming status of the handle (by value)
inline std::ostream& operator<<(std::ostream& os, hardware_interface::JointHandle rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  os << ", cmd: q" << rhs.getCommand();
  return os;
}

inline std::ostream& operator<<(std::ostream& os, hardware_interface::JointStateHandle rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  return os;
}

inline std::ostream& operator<<(std::ostream& os, hardware_interface::VelEffJointHandle rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  os << ", cmd: qd" << rhs.getCommandVelocity() << ", eff " << rhs.getCommandEffort();
  return os;
}

inline std::ostream& operator<<(std::ostream& os, hardware_interface::PosVelEffJointHandle rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ",  qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  os << ", cmd: q"<< rhs.getCommandPosition()<<", qd " << rhs.getCommandVelocity() << ", eff " << rhs.getCommandEffort();
  return os;
}




#endif  // CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H
