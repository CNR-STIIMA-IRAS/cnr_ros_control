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
  Eigen::VectorXd  q;
  Eigen::VectorXd  qd;
  Eigen::VectorXd  qdd;
  Eigen::VectorXd  effort;
  void resize(const size_t& nAx)
  {
    q     .resize(nAx);
    qd    .resize(nAx);
    qdd   .resize(nAx);
    effort.resize(nAx);
    setZero();
  }
  void setZero()
  {
    q     .setZero();
    qd    .setZero();
    qdd   .setZero();
    effort.setZero();
  }
  KinematicStatus() = default;
  KinematicStatus(const KinematicStatus& cpy)
  {
    q     = q     ;
    qd    = qd    ;
    qdd   = qdd   ;
    effort= effort;
  }
  KinematicStatus& operator=(const KinematicStatus& rhs)
  {
    this->q     = rhs.q     ;
    this->qd    = rhs.qd    ;
    this->qdd   = rhs.qdd   ;
    this->effort= rhs.effort;
    return *this;
  }
};


typedef std::shared_ptr<KinematicStatus> KinematicStatusPtr;
typedef const std::shared_ptr<KinematicStatus const> KinematicStatusConstPtr;


//template<class T>
//bool extract(const T* hw, KinematicStatus& st)
//{
//  return false;
//}


//template<class T>
//bool extract(const KinematicStatus& cmd, T* hw)
//{
//  return false;
//}


//template<class T>
//bool extract(const T* hw, KinematicStatusPtr st)
//{
//  return false;
//}


//template<class T>
//bool extract(const KinematicStatusPtr cmd, T* hw)
//{
//  return false;
//}



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
  rosdyn::ChainPtr       getChain  ( const std::string& from, const std::string& to)
  {
    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.806;
    rosdyn::ChainPtr chain = rosdyn::createChain(*m_model,from,to,gravity);
    return chain;
  }

  void updateTransformation(const KinematicStatus& state)
  {
    m_Tbt   = m_chain->getTransformation(state.q);
    m_J     = m_chain->getJacobian(state.q);
    m_twist = m_J * state.qd;
  }
  bool init(cnr_logger::TraceLoggerPtr logger, ros::NodeHandle& root_nh, ros::NodeHandle& ctrl_nh)
  {
    XmlRpc::XmlRpcValue value;
    if (!ctrl_nh.getParam("controlled_joint", value) && !ctrl_nh.getParam("controlled_joints", value))
    {
      CNR_RETURN_FALSE(*logger, "The param " + ctrl_nh.getNamespace() + "/controlled_joint(s) not defined");
    }

    if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      if (value[0].getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        for (int i = 0; i < value.size(); i++)
        {
          m_joint_names.push_back((std::string)(value[i]));
        }
      }
      else
      {
        CNR_RETURN_FALSE(*logger, "The param " + ctrl_nh.getNamespace() + "/controlled_joint(s) bad formed");
      }
    }
    else if (value.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      m_joint_names.push_back((std::string)(value));
    }
    else
    {
      CNR_RETURN_FALSE(*logger,  "The param " + ctrl_nh.getNamespace() + "/controlled_joint(s) bad formed");
    }

    m_nAx = m_joint_names.size();

    std::string robot_description_param;
    std::string robot_description;
    if (!ctrl_nh.getParam("robot_description_param", robot_description_param ) )
    {
      if (!root_nh.getParam("robot_description_param", robot_description_param ) )
      {
        CNR_WARN(*logger, ctrl_nh.getNamespace() + "/robot_description_param/ is not in rosparam server. Superimposed defualt value '/robot_description'");
        robot_description_param = "/robot_description";
      }
    }
    if (!ctrl_nh.getParam(robot_description_param, robot_description))
    {
      CNR_FATAL(*logger, "Parameter '/robot_description' does not exist");
      CNR_RETURN_FALSE(*logger);
    }
    m_model = urdf::parseURDF(robot_description);

    m_upper_limit    .resize(m_nAx); m_upper_limit  .setZero();
    m_lower_limit    .resize(m_nAx); m_lower_limit  .setZero();
    m_qd_limit       .resize(m_nAx); m_qd_limit     .setZero();
    m_qdd_limit      .resize(m_nAx); m_qdd_limit    .setZero();
    for (unsigned int iAx = 0; iAx < m_nAx; iAx++)
    {
      try
      {
          m_upper_limit(iAx) = m_model->getJoint(m_joint_names.at(iAx))->limits->upper;
          m_lower_limit(iAx) = m_model->getJoint(m_joint_names.at(iAx))->limits->lower;

          if ((m_upper_limit(iAx) == 0) && (m_lower_limit(iAx) == 0))
          {
            m_upper_limit(iAx) = std::numeric_limits<double>::infinity();
            m_lower_limit(iAx) = -std::numeric_limits<double>::infinity();
            ROS_INFO("upper and lower limits are both equal to 0, set +/- infinity");
          }

          bool has_velocity_limits;
          if (!ctrl_nh.getParam(
                "/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/has_velocity_limits", has_velocity_limits))
          {
            has_velocity_limits = false;
          }
          bool has_acceleration_limits;
          if (!ctrl_nh.getParam(
                "/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/has_acceleration_limits", has_acceleration_limits))
          {
            has_acceleration_limits = false;
          }

          m_qd_limit(iAx) = m_model->getJoint(m_joint_names.at(iAx))->limits->velocity;
          if (has_velocity_limits)
          {
            double vel;
            if (!ctrl_nh.getParam("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_velocity", vel))
            {
              ROS_ERROR_STREAM("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_velocity is not defined");
              return false;
            }
            if (vel < m_qd_limit(iAx))
              m_qd_limit(iAx) = vel;
          }

          if (has_acceleration_limits)
          {
            double acc;
            if (!ctrl_nh.getParam("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_acceleration", acc))
            {
              ROS_ERROR_STREAM("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_acceleration is not defined");
              return false;
            }
            m_qdd_limit(iAx) = acc;
          }
          else
            m_qdd_limit(iAx) = 10 * m_qd_limit(iAx);
      }
      catch (...)
      {
        CNR_RETURN_FALSE(*logger,
          "Controller '" + ctrl_nh.getNamespace() + "' failed in init. " + std::string("")
          + "The controlled joint named '" + m_joint_names.at(iAx) + "' is not managed by hardware_interface");
      }
    }

    if (!ctrl_nh.getParam("base_link", m_base_link ) )
    {
      if (!root_nh.getParam("base_link", m_base_link ) )
      {
        CNR_ERROR(*logger, "'Neither '" + ctrl_nh.getNamespace() + "/base_link' " +
                           "nor '"      + root_nh.getNamespace() + "/base_link' are not in rosparam server.");
        CNR_RETURN_FALSE(*logger);
      }
    }

    if (!ctrl_nh.getParam("tool_link", m_tool_link ) )
    {
      if (!root_nh.getParam("tool_link", m_tool_link ) )
      {
        CNR_ERROR(*logger, "'Neither '" + ctrl_nh.getNamespace() + "/tool_link' " +
                           "nor '"      + root_nh.getNamespace() + "/tool_link' are not in rosparam server.");
        CNR_RETURN_FALSE(*logger);
      }
    }

    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.806;

    std::shared_ptr<rosdyn::Link> root_link(new rosdyn::Link);  //link primitivo da cui parte la catena cinematica (world ad esempio)
    root_link->fromUrdf(m_model->root_link_);

    m_chain.reset(new rosdyn::Chain(root_link, m_base_link, m_tool_link, gravity)); //ricostruisce tutta la catena cinematica andando a leggere l'URDF
    m_chain->setInputJointsName(m_joint_names);
    m_link_names = m_chain->getLinksName( );

    CNR_RETURN_TRUE(*logger);
  }
};

typedef std::shared_ptr<KinematicsStruct> KinematicsStructPtr;
typedef const std::shared_ptr<KinematicsStruct const> KinematicsStructConstPtr;

}
#endif  // CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H



