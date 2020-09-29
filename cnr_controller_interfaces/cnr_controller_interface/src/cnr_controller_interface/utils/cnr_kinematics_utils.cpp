#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>

namespace cnr_controller_interface
{


void KinematicStatus::resize(const std::vector<std::string>& names)
{
  this->joint_names = names;
  this->q     .resize(joint_names.size());
  this->qd    .resize(joint_names.size());
  this->qdd   .resize(joint_names.size());
  this->effort.resize(joint_names.size());
  this->setZero();
}

void KinematicStatus::setZero()
{
  this->q     .setZero();
  this->qd    .setZero();
  this->qdd   .setZero();
  this->effort.setZero();
}

KinematicStatus::KinematicStatus(const KinematicStatus& cpy)
{
  this->joint_names = cpy.joint_names;
  this->q           = q     ;
  this->qd          = qd    ;
  this->qdd         = qdd   ;
  this->effort      = effort;
}

KinematicStatus& KinematicStatus::operator=(const KinematicStatus& rhs)
{
  this->joint_names = rhs.joint_names;
  this->q           = rhs.q     ;
  this->qd          = rhs.qd    ;
  this->qdd         = rhs.qdd   ;
  this->effort      = rhs.effort;
  return *this;
}


rosdyn::ChainPtr  KinematicsStruct::getChain  ( const std::string& from, const std::string& to)
{
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;
  rosdyn::ChainPtr chain = rosdyn::createChain(*m_model,from,to,gravity);
  return chain;
}

void KinematicsStruct::updateTransformation(const KinematicStatus& state)
{
  m_Tbt   = m_chain->getTransformation(state.q);
  m_J     = m_chain->getJacobian(state.q);
  m_twist = m_J * state.qd;
}

bool KinematicsStruct::init(cnr_logger::TraceLoggerPtr logger, ros::NodeHandle& root_nh, ros::NodeHandle& ctrl_nh)
{

  try
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
          auto joint_model = m_model->getJoint(m_joint_names.at(iAx));
          if(!joint_model)
          {
            CNR_RETURN_FALSE(*logger,
              "Controller '" + ctrl_nh.getNamespace() + "' failed in init. " + std::string("")
              + "The controlled joint named '" + m_joint_names.at(iAx) + "' is not managed by hardware_interface");
          }
          m_upper_limit(iAx) = joint_model->limits->upper;
          m_lower_limit(iAx) = joint_model->limits->lower;

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

          m_qd_limit(iAx) = joint_model->limits->velocity;
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
  catch(std::exception& e)
  {
    CNR_RETURN_FALSE(*logger, "Error caught: " + std::string(e.what()) );
  }
}


}



