#ifndef CNR_CONTROLLER_INTERFACE__CNR_HANDLES__H
#define CNR_CONTROLLER_INTERFACE__CNR_HANDLES__H

#include <map>
#include <rosdyn_chain_state/chain_state.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>

namespace cnr
{
namespace control
{

inline const rosdyn::ChainState* const getPtr(const rosdyn::ChainState& in)
{
  return &in;
}

inline rosdyn::ChainState* getPtr(rosdyn::ChainState& in)
{
  return &in;
}

typedef std::map<std::string, size_t> HandleIndexes;
typedef std::shared_ptr<HandleIndexes> HandleIndexesPtr;
typedef const std::shared_ptr<HandleIndexes const> HandleIndexesConstPtr;

HandleIndexes get_index_map(const std::vector<std::string>& names, const rosdyn::Chain& ks);

class HandlerBase
{
protected:
  bool initialized_ = false;
  HandleIndexes indexes_;

  template<class H>
  void init(const std::map<std::string, H>& resources, const rosdyn::Chain& chain)
  {
    std::vector<std::string> names(resources.size());
    std::transform(resources.begin(), resources.end(), names.begin(), [](const std::pair<std::string, H>& p) { return p.first; });
    indexes_ = get_index_map(names,chain);
    initialized_ = true;
  }

public:
  virtual void init(const rosdyn::Chain& chain) = 0; 
};


template<class Handle, class HardwareInterface>
struct Handler : public HandlerBase
{
  std::map<std::string, Handle> handles_;

  void init(const rosdyn::Chain& chain) { HandlerBase::init(handles_, chain); }
  void flush(rosdyn::ChainState& /*ks*/)  {}
  void update(const rosdyn::ChainState& /*ks*/) {}
};



/**
 * JointStateInterface
 */
template<>
struct Handler<hardware_interface::JointStateHandle, hardware_interface::JointStateInterface> : public HandlerBase
{
  std::map<std::string, hardware_interface::JointStateHandle> handles_;

  void init(const rosdyn::Chain& chain) { HandlerBase::init(handles_, chain); }
  void flush(rosdyn::ChainState& ks)
  {
    if(!initialized_) 
      throw std::runtime_error("Handler must be initialized!");

    for(auto const & ax : indexes_)
    {
      ks.q(ax.second) = handles_.at(ax.first).getPosition();
      ks.qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks.qdd(ax.second) = 0.0;
      ks.effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }

  void update(const rosdyn::ChainState& /*ks*/)
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

  void init(const rosdyn::Chain& chain) { HandlerBase::init(handles_, chain); }
  void flush(rosdyn::ChainState& ks)
  {
    if(!initialized_) 
      throw std::runtime_error("Handler must be initialized!");

    for(auto const & ax : indexes_)
    {
      ks.q(ax.second) = handles_.at(ax.first).getPosition();
      ks.qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks.qdd(ax.second) = 0.0;
      ks.effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }

  void update(const rosdyn::ChainState& ks)
  {
    if(!initialized_) 
      throw std::runtime_error("Handler must be initialized!");

    for(auto const & ax : indexes_)
    {
      handles_.at(ax.first).setCommandVelocity(ks.qd(ax.second));
      handles_.at(ax.first).setCommandEffort(ks.effort(ax.second));
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

  void init(const rosdyn::Chain& chain) { HandlerBase::init(handles_, chain); }
  void flush(rosdyn::ChainState& ks)
  {
    if(!initialized_) 
      throw std::runtime_error("Handler must be initialized!");

    for(auto const & ax : indexes_)
    {
      ks.q(ax.second) = handles_.at(ax.first).getPosition();
      ks.qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks.qdd(ax.second) = 0.0;
      ks.effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }


  void update(const rosdyn::ChainState& ks)
  {
    if(!initialized_) 
      throw std::runtime_error("Handler must be initialized!");

    for(auto const & ax : indexes_)
    {
      size_t index = ax.second;
      handles_.at(ax.first).setCommandPosition(ks.q(index));
      handles_.at(ax.first).setCommandVelocity(ks.qd(index));
      handles_.at(ax.first).setCommandEffort  (ks.effort(index));
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

  void init(const rosdyn::Chain& chain) { HandlerBase::init(handles_, chain); }
  void flush(rosdyn::ChainState& ks)
  {
    if(!initialized_) 
      throw std::runtime_error("Handler must be initialized!");

    for(auto const & ax : indexes_)
    {
      ks.q(ax.second) = handles_.at(ax.first).getPosition();
      ks.qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks.qdd(ax.second) = 0.0;
      ks.effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }

  void update(const rosdyn::ChainState& ks)
  {
    if(!initialized_) 
      throw std::runtime_error("Handler must be initialized!");

    for(auto const & ax : indexes_)
    {
      handles_.at(ax.first).setCommand(ks.q(ax.second));
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

  void init(const rosdyn::Chain& chain) { HandlerBase::init(handles_, chain); }
  void flush(rosdyn::ChainState& ks)
  {
    if(!initialized_) 
      throw std::runtime_error("Handler must be initialized!");

    for(auto const & ax : indexes_)
    {
      ks.q(ax.second) = handles_.at(ax.first).getPosition();
      ks.qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks.qdd(ax.second) = 0.0;
      ks.effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }

  void update(const rosdyn::ChainState& ks)
  {
    if(!initialized_) 
      throw std::runtime_error("Handler must be initialized!");

    for(auto const & ax : indexes_)
    {
      handles_.at(ax.first).setCommand(ks.effort(ax.second));
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

  void init(const rosdyn::Chain& chain) { HandlerBase::init(handles_, chain); }
  void flush(rosdyn::ChainState& ks)
  {
    if(!initialized_) 
      throw std::runtime_error("Handler must be initialized!");

    for(auto const & ax : indexes_)
    {
      ks.q(ax.second) = handles_.at(ax.first).getPosition();
      ks.qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks.qdd(ax.second) = 0.0;
      ks.effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }


  void update(const rosdyn::ChainState& ks)
  {
    if(!initialized_) 
      throw std::runtime_error("Handler must be initialized!");
    for(auto const & ax : indexes_)
    {
      handles_.at(ax.first).setCommand(ks.qd(ax.second));
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

  void init(const rosdyn::Chain& chain) { HandlerBase::init(handles_, chain); }
  void flush(rosdyn::ChainState& ks)
  {
    if(!initialized_) 
      throw std::runtime_error("Handler must be initialized!");

    for(auto const & ax : indexes_)
    {
      ks.q(ax.second) = handles_.at(ax.first).getPosition();
      ks.qd(ax.second) = handles_.at(ax.first).getVelocity();
      ks.qdd(ax.second) = 0.0;
      ks.effort(ax.second) = handles_.at(ax.first).getEffort();
    }
  }

  void update(const rosdyn::ChainState& ks)
  {
    if(!initialized_) 
      throw std::runtime_error("Handler must be initialized!");
      
    for(auto const & ax : indexes_)
    {
      handles_.at(ax.first).setCommand(ks.q(ax.second));
    }
  }
};

}  // namespace control
}  // namespace ros

namespace std
{
// streaming status of the handle (by reference)
std::string to_string(const hardware_interface::JointHandle& rhs);
std::string to_string(const hardware_interface::JointStateHandle& rhs);
std::string to_string(const hardware_interface::VelEffJointHandle& rhs);
std::string to_string(const hardware_interface::PosVelEffJointHandle& rhs);

template<class Handle, class HardwareInterface>
inline std::string to_string(const cnr::control::Handler<Handle, HardwareInterface>& handler)
{
  std::string ret;
  for(const auto & hh: handler.handles_)
  {
    ret += to_string(hh.second)+"\n";
  }
  return ret;
}
}

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

#endif  // CNR_CONTROLLER_INTERFACE__CNR_HANDLES__H
