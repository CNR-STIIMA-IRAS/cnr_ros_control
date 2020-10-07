#ifndef CNR_CONTROLLER_INTERFACE__CNR_KINEMATIC_STATUS_H
#define CNR_CONTROLLER_INTERFACE__CNR_KINEMATIC_STATUS_H

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <cnr_logger/cnr_logger.h>
#include <Eigen/Core>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <rosdyn_core/primitives.h>

namespace cnr_controller_interface
{

class KinematicsStruct
{

private:
  urdf::ModelInterfaceSharedPtr m_model;
  std::string                   m_base_link;
  std::string                   m_tool_link;
  rosdyn::ChainPtr              m_chain;

  std::vector<std::string>      m_joint_names;
  std::vector<std::string>      m_link_names;
  size_t                        m_nAx;
  Eigen::VectorXd               m_upper_limit;
  Eigen::VectorXd               m_lower_limit;
  Eigen::VectorXd               m_qd_limit;
  Eigen::VectorXd               m_qdd_limit;
  Eigen::VectorXd               m_effort_limit;

  Eigen::IOFormat               m_cfrmt;

public:
  const size_t& nAx                         ( ) const { return m_nAx;         }
  const Eigen::VectorXd& upperLimit         ( ) const { return m_upper_limit; }
  const Eigen::VectorXd& lowerLimit         ( ) const { return m_lower_limit; }
  const Eigen::VectorXd& speedLimit         ( ) const { return m_qd_limit;    }
  const Eigen::VectorXd& accelerationLimit  ( ) const { return m_qdd_limit;   }
  const Eigen::VectorXd& effortLimit        ( ) const { return m_effort_limit;}
  const std::vector<std::string>& jointNames( ) const { return m_joint_names; }
  const double& upperLimit         (size_t iAx) const { return m_upper_limit(iAx); }
  const double& lowerLimit         (size_t iAx) const { return m_lower_limit(iAx); }
  const double& speedLimit         (size_t iAx) const { return m_qd_limit(iAx);    }
  const double& accelerationLimit  (size_t iAx) const { return m_qdd_limit(iAx);   }
  const double& eeffortLimit       (size_t iAx) const { return m_effort_limit(iAx);   }
  const std::string& jointName     (size_t iAx) const { return m_joint_names.at(iAx); }
  double upperLimit         (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_upper_limit (idx); }
  double lowerLimit         (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_lower_limit (idx); }
  double speedLimit         (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_qd_limit    (idx); }
  double accelerationLimit  (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_qdd_limit   (idx); }
  double eeffortLimit       (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_effort_limit(idx); }
  int jointIndex (const std::string& name) const
  {
    auto it = std::find(m_joint_names.begin(),m_joint_names.end(), name);
    return it == m_joint_names.end() ? -1 : std::distance(m_joint_names.begin(), it);
  }

  const std::vector<std::string>&    linkNames ( ) const { return m_link_names;}
  const std::string&                 baseLink  ( ) const { return m_base_link; }
  const std::string&                 baseFrame ( ) const { return baseLink();  }
  const std::string&                 toolLink  ( ) const { return m_tool_link; }
  const std::string&                 toolFrame ( ) const { return toolLink();  }
  
  rosdyn::ChainPtr       getChain  ( )                { return m_chain;     }
  rosdyn::ChainPtr       getChain  ( const std::string& from, const std::string& to);

  bool init(cnr_logger::TraceLoggerPtr logger, ros::NodeHandle& root_nh, ros::NodeHandle& ctrl_nh);


  bool saturateSpeed(Eigen::Ref<Eigen::VectorXd> qd_target,
                     const Eigen::Ref<const Eigen::VectorXd> qd_actual,
                     const Eigen::Ref<const Eigen::VectorXd> q_actual,
                     double dt,
                     double max_velocity_multiplier,
                     bool preserve_direction,
                     std::stringstream* report);

  bool saturateSpeed(Eigen::Ref<Eigen::VectorXd> qd_target,
                     const Eigen::Ref<const Eigen::VectorXd> qd_actual,
                     double dt,
                     double max_velocity_multiplier,
                     bool preserve_direction,
                     std::stringstream* report);

  bool saturateSpeed(Eigen::Ref<Eigen::VectorXd> qd_target,
                     double max_velocity_multiplier,
                     bool preserve_direction,
                     std::stringstream* report);

  bool saturatePosition(Eigen::Ref<Eigen::VectorXd> q_target, std::stringstream* report);

};

typedef std::shared_ptr<KinematicsStruct> KinematicsStructPtr;
typedef const std::shared_ptr<KinematicsStruct const> KinematicsStructConstPtr;




class KinematicStatus
{
private:
  Eigen::VectorXd               q_;
  Eigen::VectorXd               qd_;
  Eigen::VectorXd               qdd_;
  Eigen::VectorXd               effort_;
  
  Eigen::Affine3d               Tbt_;
  Eigen::Matrix<double, 6, 1>   twist_;
  Eigen::Matrix<double, 6, 1>   twistd_;
  Eigen::Matrix6Xd              jacobian_;

  int  index (const std::string& name) const
  {
    auto it = std::find(kin_->jointNames().begin(), kin_->jointNames().end(), name);
    return (it == kin_->jointNames().end()) ? -1 : std::distance(kin_->jointNames().begin(),it);
  }
  
  KinematicsStructPtr kin_;
public:
  
  typedef std::shared_ptr<KinematicStatus> Ptr;
  typedef const std::shared_ptr<KinematicStatus const> ConstPtr;

  // GETTER
  const std::string&               jointName  (const size_t iAx) const { return kin_->jointNames().at(iAx);}
  const std::vector<std::string>&  jointNames () const { return kin_->jointNames();}
  const size_t                     nAx        () const { return kin_->nAx(); }
  const Eigen::VectorXd&           q          () const { return q_;          }
  const Eigen::VectorXd&           qd         () const { return qd_;         }
  const Eigen::VectorXd&           qdd        () const { return qdd_;        }
  const Eigen::VectorXd&           effort     () const { return effort_;     }

  const double& q      (const size_t& iAx) const { if(iAx>=nAx()) throw std::runtime_error("Index out of range!");return q_     (iAx); }
  const double& qd     (const size_t& iAx) const { if(iAx>=nAx()) throw std::runtime_error("Index out of range!");return qd_    (iAx); }
  const double& qdd    (const size_t& iAx) const { if(iAx>=nAx()) throw std::runtime_error("Index out of range!");return qdd_   (iAx); }
  const double& effort (const size_t& iAx) const { if(iAx>=nAx()) throw std::runtime_error("Index out of range!");return effort_(iAx); }

  const double& q     (const std::string& name) const {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return q     (iAx); }
  const double& qd    (const std::string& name) const {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return qd    (iAx); }
  const double& qdd   (const std::string& name) const {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return qdd   (iAx); }
  const double& effort(const std::string& name) const {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return effort(iAx); }

  // SETTER
  Eigen::VectorXd&  q          () { return q_;          }
  Eigen::VectorXd&  qd         () { return qd_;         }
  Eigen::VectorXd&  qdd        () { return qdd_;        }
  Eigen::VectorXd&  effort     () { return effort_;     }

  double& q      (const size_t& iAx) { return q_     (iAx); }
  double& qd     (const size_t& iAx) { return qd_    (iAx); }
  double& qdd    (const size_t& iAx) { return qdd_   (iAx); }
  double& effort (const size_t& iAx) { return effort_(iAx); }

  double& q     (const std::string& name) {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return q     (iAx); }
  double& qd    (const std::string& name) {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return qd    (iAx); }
  double& qdd   (const std::string& name) {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return qdd   (iAx); }
  double& effort(const std::string& name) {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return effort(iAx); }
  
  KinematicsStructPtr getKin() const { return kin_; }
  KinematicStatus& updateTransformation( );
  
  const Eigen::Affine3d&             toolPose  ( ) const { return Tbt_;       }
  const Eigen::Matrix<double, 6, 1>& twist     ( ) const { return twist_;     }
  const Eigen::Matrix<double, 6, 1>& twistd    ( ) const { return twistd_;    }
  const Eigen::Matrix6Xd             jacobian  ( ) const { return jacobian_;  }

  KinematicStatus() = delete;
  KinematicStatus(KinematicsStructPtr kin);
  KinematicStatus(const KinematicStatus& cpy);
  KinematicStatus& operator=(const KinematicStatus& rhs);

  void setZero();
};

typedef KinematicStatus::Ptr KinematicStatusPtr;
typedef KinematicStatus::ConstPtr KinematicStatusConstPtr;



}
#endif  // CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H



