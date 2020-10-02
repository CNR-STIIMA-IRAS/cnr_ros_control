#ifndef CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H
#define CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H

#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>

namespace cnr_controller_interface
{

const KinematicStatus * const getPtr( const KinematicStatusConstPtr& in);
const KinematicStatus * const getPtr( const KinematicStatus& in);
KinematicStatus* getPtr( KinematicStatusPtr& in);
KinematicStatus* getPtr( KinematicStatus& in);


/**
 * JointStateInterface
 */
bool get_from_hw(hardware_interface::JointStateInterface* hi, KinematicStatus* ks);

/**
 * hardware_interface::VelEffJointInterface
 */
bool get_from_hw(hardware_interface::VelEffJointInterface* const hi, KinematicStatus* ks);
bool set_to_hw  (const KinematicStatus * const ks, hardware_interface::VelEffJointInterface*  hi);

/**
 * PosVelEffJointInterface
 */
bool get_from_hw(hardware_interface::PosVelEffJointInterface* const hi, KinematicStatus* ks);
bool set_to_hw  (const KinematicStatus * const ks, hardware_interface::PosVelEffJointInterface* hi);

/**
 * JointCommandInterface
 */

bool get_from_hw(hardware_interface::JointCommandInterface* const in, KinematicStatus* ks);
bool set_to_hw(const KinematicStatus* const  ks,hardware_interface::JointCommandInterface* hi);

/**
 * @brief EffortJointInterface
 */
bool get_from_hw(hardware_interface::EffortJointInterface* const ki, KinematicStatus * ks);
bool set_to_hw(const KinematicStatus * const ks, hardware_interface::EffortJointInterface* hi);

/**
 * @brief VelocityJointInterface
 */
bool get_from_hw(hardware_interface::VelocityJointInterface* const hi, KinematicStatus * ks);
bool set_to_hw  (const KinematicStatus* const ks, hardware_interface::VelocityJointInterface* hi);

/**
 * @brief VelocityJointInterface
 */

bool get_from_hw(hardware_interface::PositionJointInterface* const hi,KinematicStatus* ks);
bool set_to_hw  (const KinematicStatus* const ks, hardware_interface::PositionJointInterface* hi);

}  // namespace hardware_interface




// streaming status of the handle (by reference)
inline std::ostream& operator<<(std::ostream& os, hardware_interface::JointHandle& rhs)
{
  os << rhs.getName() << ", state: q" << rhs.getPosition() << ", qd" << rhs.getVelocity() << ", eff " << rhs.getEffort();
  os << ", cmd: " << rhs.getCommand();
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
  os << ", cmd: " << rhs.getCommand();
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
