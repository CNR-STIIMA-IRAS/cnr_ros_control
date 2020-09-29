#ifndef CNR_CONTROLLER_INTERFACE__CNR_KINEMATIC_STATUS_H
#define CNR_CONTROLLER_INTERFACE__CNR_KINEMATIC_STATUS_H

#include <cnr_logger/cnr_logger.h>
#include <eigen3/Eigen/Core>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <rosdyn_core/primitives.h>

namespace cnr_controller_interface
{

struct KinematicStatus
{
  std::vector<std::string> joint_names;
  Eigen::VectorXd  q;
  Eigen::VectorXd  qd;
  Eigen::VectorXd  qdd;
  Eigen::VectorXd  effort;

  KinematicStatus() = default;
  KinematicStatus(const KinematicStatus& cpy);
  KinematicStatus& operator=(const KinematicStatus& rhs);

  void resize(const std::vector<std::string>& names);
  void setZero();
};

typedef std::shared_ptr<KinematicStatus> KinematicStatusPtr;
typedef const std::shared_ptr<KinematicStatus const> KinematicStatusConstPtr;


class KinematicsStruct
{

private:
  urdf::ModelInterfaceSharedPtr m_model;
  std::string                   m_base_link;
  std::string                   m_tool_link;
  rosdyn::ChainPtr              m_chain;
  Eigen::Affine3d               m_Tbt;
  Eigen::Matrix<double, 6, 1>   m_twist;
  Eigen::Matrix6Xd              m_J;

  std::vector<std::string>      m_joint_names;
  std::vector<std::string>      m_link_names;
  size_t                        m_nAx;
  Eigen::VectorXd               m_upper_limit;
  Eigen::VectorXd               m_lower_limit;
  Eigen::VectorXd               m_qd_limit;
  Eigen::VectorXd               m_qdd_limit;
  Eigen::VectorXd               m_effort_limit;

public:
  const size_t& nAx                         ( ) const { return m_nAx; }
  const Eigen::VectorXd& upperLimit         ( ) const { return m_upper_limit; }
  const Eigen::VectorXd& lowerLimit         ( ) const { return m_lower_limit; }
  const Eigen::VectorXd& speedLimit         ( ) const { return m_qd_limit;    }
  const Eigen::VectorXd& accelerationLimit  ( ) const { return m_qdd_limit;   }
  const Eigen::VectorXd& effortLimit        ( ) const { return m_effort_limit;   }
  const std::vector<std::string>& jointNames( ) const { return m_joint_names; }
  const double& upperLimit         (size_t iAx) const { return m_upper_limit(iAx); }
  const double& lowerLimit         (size_t iAx) const { return m_lower_limit(iAx); }
  const double& speedLimit         (size_t iAx) const { return m_qd_limit(iAx);    }
  const double& accelerationLimit  (size_t iAx) const { return m_qdd_limit(iAx);   }
  const double& eeffortLimit       (size_t iAx) const { return m_effort_limit(iAx);   }
  const std::string& jointName     (size_t iAx) const { return m_joint_names.at(iAx); }
  const std::vector<std::string>& linkNames ( ) const { return m_link_names;}
  const std::string& baseLink      ( )          const { return m_base_link; }
  const std::string& baseFrame     ( )          const { return baseLink();  }
  const std::string& toolLink      ( )          const { return m_tool_link; }
  const std::string& toolFrame     ( )          const { return toolLink();  }
  const Eigen::Affine3d& toolPose  ( )          const { return m_Tbt;       }
  rosdyn::ChainPtr       getChain  ( )                { return m_chain;     }
  rosdyn::ChainPtr       getChain  ( const std::string& from, const std::string& to);
  void updateTransformation(const KinematicStatus& state);
  bool init(cnr_logger::TraceLoggerPtr logger, ros::NodeHandle& root_nh, ros::NodeHandle& ctrl_nh);
};

typedef std::shared_ptr<KinematicsStruct> KinematicsStructPtr;
typedef const std::shared_ptr<KinematicsStruct const> KinematicsStructConstPtr;

}
#endif  // CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H



