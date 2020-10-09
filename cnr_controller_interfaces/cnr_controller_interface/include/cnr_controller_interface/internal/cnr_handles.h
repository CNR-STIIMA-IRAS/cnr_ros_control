#ifndef CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H
#define CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H

#include <map>
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


typedef std::map<std::string, size_t> HandleIndexes;
typedef std::shared_ptr<HandleIndexes> HandleIndexesPtr;
typedef const std::shared_ptr<HandleIndexes const> HandleIndexesConstPtr;

HandleIndexes get_index_map(const std::vector<std::string>& names, cnr_controller_interface::KinematicsStructConstPtr ks);
HandleIndexes get_index_map(const std::vector<std::string>& names, cnr_controller_interface::KinematicStatusConstPtr ks);

struct HandlerBase
{
  bool initialized_=false;
  HandleIndexes indexes_;

  template<class H>
  void init(const std::map<std::string, H>& resources, cnr_controller_interface::KinematicsStructPtr ks) 
  { 
    std::vector<std::string> names(resources.size());
    std::transform(resources.begin(), resources.end(), names.begin(), [](auto & p) { return p.first; });
    indexes_ = get_index_map(names,ks);
    initialized_ = true;
  }
  template<class H>
  void init(const std::map<std::string, H>& resources, cnr_controller_interface::KinematicStatusConstPtr ks) 
  { 
    std::vector<std::string> names(resources.size());
    std::transform(resources.begin(), resources.end(), names.begin(), [](auto & p) { return p.first; });
    indexes_ = get_index_map(names,ks);
    initialized_ = true;
  }
};


template<class Handle, class HardwareInterface>
struct Handler : public HandlerBase
{
  std::map<std::string, Handle> handles_;
    
  virtual void operator>>(cnr_controller_interface::KinematicStatusPtr ks)  {};
  virtual void operator<<(cnr_controller_interface::KinematicStatusConstPtr ks) {};
};



/**
 * JointStateInterface
 */
template<>
struct Handler<hardware_interface::JointStateHandle, hardware_interface::JointStateInterface> : public HandlerBase
{
  std::map<std::string, hardware_interface::JointStateHandle> handles_;
    
  virtual void operator>>(cnr_controller_interface::KinematicStatusPtr ks) 
  {
    if(!initialized_) init(handles_, ks);
    for(auto const & ax : indexes_)
    {
      ks->q(ax.second) = handles_.at(ax.first).getPosition();
      ks->qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks->qdd(ax.second) = 0.0;
      ks->effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }
  virtual void operator<<(cnr_controller_interface::KinematicStatusConstPtr ks)
  {
  }
};


/**
 * hardware_interface::VelEffJointInterface
 */
template<>
struct Handler<hardware_interface::VelEffJointHandle, hardware_interface::VelEffJointInterface> : public HandlerBase
{
  std::map<std::string, hardware_interface::VelEffJointHandle> handles_;
    
  virtual void operator>>(cnr_controller_interface::KinematicStatusPtr ks) 
  {
    if(!initialized_) init(handles_, ks);
    for(auto const ax : indexes_)
    {
      ks->q(ax.second) = handles_.at(ax.first).getPosition();
      ks->qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks->qdd(ax.second) = 0.0;
      ks->effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }
  virtual void operator<<(cnr_controller_interface::KinematicStatusConstPtr ks)
  {
    if(!initialized_) init(handles_, ks);
    for(auto const & ax : indexes_)
    {
      handles_.at(ax.first).setCommandVelocity(ks->qd(ax.second));
      handles_.at(ax.first).setCommandEffort(ks->effort(ax.second));
    }
  }
};

/**
 * PosVelEffJointInterface
 */
template<>
struct Handler<hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface> : public HandlerBase
{
  std::map<std::string, hardware_interface::PosVelEffJointHandle> handles_;
    
  virtual void operator>>(cnr_controller_interface::KinematicStatusPtr ks) 
  {
    if(!initialized_) init(handles_, ks);
    for(auto const & ax : indexes_)
    {
      ks->q(ax.second) = handles_.at(ax.first).getPosition();
      ks->qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks->qdd(ax.second) = 0.0;
      ks->effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }
  virtual void operator<<(cnr_controller_interface::KinematicStatusConstPtr ks)
  {
    if(!initialized_) init(handles_, ks);
    for(auto const & ax : indexes_)
    {
      handles_.at(ax.first).setCommandPosition(ks->q(ax.second));
      handles_.at(ax.first).setCommandVelocity(ks->qd(ax.second));
      handles_.at(ax.first).setCommandEffort  (ks->effort(ax.second));
    }
  }
};

/**
 * JointCommandInterface
 */
template<>
struct Handler<hardware_interface::JointHandle, hardware_interface::JointCommandInterface> : public HandlerBase
{
  std::map<std::string, hardware_interface::JointHandle> handles_;
    
  virtual void operator>>(cnr_controller_interface::KinematicStatusPtr ks) 
  {
    if(!initialized_) init(handles_, ks);
    for(auto const & ax : indexes_)
    {
      ks->q(ax.second) = handles_.at(ax.first).getPosition();
      ks->qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks->qdd(ax.second) = 0.0;
      ks->effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }
  virtual void operator<<(cnr_controller_interface::KinematicStatusConstPtr ks)
  {
    if(!initialized_) init(handles_, ks);
    for(auto const & ax : indexes_)
    {
      handles_.at(ax.first).setCommand(ks->q(ax.second));
    }
  }
};

/**
 * @brief EffortJointInterface
 */
template<>
struct Handler<hardware_interface::JointHandle, hardware_interface::EffortJointInterface> : public HandlerBase
{
  std::map<std::string, hardware_interface::JointHandle> handles_;
    
  virtual void operator>>(cnr_controller_interface::KinematicStatusPtr ks) 
  {
    if(!initialized_) init(handles_, ks);
    for(auto const & ax : indexes_)
    {
      ks->q(ax.second) = handles_.at(ax.first).getPosition();
      ks->qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks->qdd(ax.second) = 0.0;
      ks->effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }
  virtual void operator<<(cnr_controller_interface::KinematicStatusConstPtr ks)
  {
    if(!initialized_) init(handles_, ks);
    for(auto const & ax : indexes_)
    {
      handles_.at(ax.first).setCommand(ks->effort(ax.second));
    }
  }
};

/**
 * @brief VelocityJointInterface
 */
template<>
struct Handler<hardware_interface::JointHandle, hardware_interface::VelocityJointInterface> : public HandlerBase
{
  std::map<std::string, hardware_interface::JointHandle> handles_;
    
  virtual void operator>>(cnr_controller_interface::KinematicStatusPtr ks) 
  {
    if(!initialized_) init(handles_, ks);
    for(auto const & ax : indexes_)
    {
      ks->q(ax.second) = handles_.at(ax.first).getPosition();
      ks->qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks->qdd(ax.second) = 0.0;
      ks->effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }
  virtual void operator<<(cnr_controller_interface::KinematicStatusConstPtr ks)
  {
    if(!initialized_) init(handles_, ks);
    for(auto const & ax : indexes_)
    {
      handles_.at(ax.first).setCommand(ks->qd(ax.second));
    }
  }
};

/**
 * @brief PositionJointInterface
 */
template<>
struct Handler<hardware_interface::JointHandle, hardware_interface::PositionJointInterface> : public HandlerBase
{
  std::map<std::string, hardware_interface::JointHandle> handles_;
    
  virtual void operator>>(cnr_controller_interface::KinematicStatusPtr ks) 
  {
    if(!initialized_) init(handles_, ks);
    for(auto const & ax : indexes_)
    {
      ks->q(ax.second) = handles_.at(ax.first).getPosition();
      ks->qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks->qdd(ax.second) = 0.0;
      ks->effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }
  virtual void operator<<(cnr_controller_interface::KinematicStatusConstPtr ks)
  {
    if(!initialized_) init(handles_, ks);
    for(auto const & ax : indexes_)
    {
      handles_.at(ax.first).setCommand(ks->q(ax.second));
    }
  }
};

}  // namespace hardware_interface


// streaming status of the handle (by reference)
std::ostream& operator<<(std::ostream& os, hardware_interface::JointHandle& rhs);
std::ostream& operator<<(std::ostream& os, hardware_interface::JointStateHandle& rhs);
std::ostream& operator<<(std::ostream& os, hardware_interface::VelEffJointHandle& rhs);
std::ostream& operator<<(std::ostream& os, hardware_interface::PosVelEffJointHandle& rhs);

// streaming status of the handle (by value)
std::ostream& operator<<(std::ostream& os, hardware_interface::JointHandle rhs);
std::ostream& operator<<(std::ostream& os, hardware_interface::JointStateHandle rhs);
std::ostream& operator<<(std::ostream& os, hardware_interface::VelEffJointHandle rhs);
std::ostream& operator<<(std::ostream& os, hardware_interface::PosVelEffJointHandle rhs);

#endif  // CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H
