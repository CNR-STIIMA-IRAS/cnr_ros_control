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

struct KinematicStatus
{
  std::vector<std::string> joint_names;
  Eigen::VectorXd  q;
  Eigen::VectorXd  qd;
  Eigen::VectorXd  qdd;
  Eigen::VectorXd  effort;

  const std::string&               get_joint_name (const size_t iAx) const { return joint_names.at(iAx);}
  const std::vector<std::string>&  get_joint_names() const { return joint_names;}
  const Eigen::VectorXd&           get_q          () const { return q;          }
  const Eigen::VectorXd&           get_qd         () const { return qd;         }
  const Eigen::VectorXd&           get_qdd        () const { return qdd;        }
  const Eigen::VectorXd&           get_effort     () const { return effort;     }

  const double&  get_q           (const size_t& iAx) const { return q     (iAx); }
  const double&  get_qd          (const size_t& iAx) const { return qd    (iAx); }
  const double&  get_qdd         (const size_t& iAx) const { return qdd   (iAx); }
  const double&  get_effort      (const size_t& iAx) const { return effort(iAx); }

  int  get_index (const std::string& name) const
  {
    auto it = std::find(joint_names.begin(), joint_names.end(), name);
    return (it == joint_names.end()) ? -1 : std::distance(joint_names.begin(),it);
  }

  double get_q     (const std::string& name) const {int iAx = get_index(name); return iAx==-1 ? NAN : q     (iAx); }
  double get_qd    (const std::string& name) const {int iAx = get_index(name); return iAx==-1 ? NAN : qd    (iAx); }
  double get_qdd   (const std::string& name) const {int iAx = get_index(name); return iAx==-1 ? NAN : qdd   (iAx); }
  double get_effort(const std::string& name) const {int iAx = get_index(name); return iAx==-1 ? NAN : effort(iAx); }


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

  Eigen::IOFormat               m_cfrmt;

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

}
#endif  // CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H



