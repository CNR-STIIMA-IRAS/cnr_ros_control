#include <sstream>
#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>


#define SP std::fixed  << std::setprecision(5)
#define TP(X) std::fixed << std::setprecision(5) << X.format(m_cfrmt)

namespace cnr_controller_interface
{



rosdyn::ChainPtr  KinematicsStruct::getChain  ( const std::string& from, const std::string& to)
{
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;
  rosdyn::ChainPtr chain = rosdyn::createChain(*m_model,from,to,gravity);
  return chain;
}

void get_joint_names(ros::NodeHandle* nh, std::vector<std::string>& names)
{
  std::vector<std::string> alternative_keys =
    {"controlled_resources", "controlled_resource", "controlled_joints", "controlled_joint", "joint_names", "joint_name"};

  names.clear();
  for(auto const & key : alternative_keys)
  {
    if(!nh->getParam(key, names))
    {
      std::string joint_name;
      if(nh->getParam(key, joint_name))
      {
        names.push_back(joint_name);
      }
    }
  }
  return;
}

bool KinematicsStruct::init(cnr_logger::TraceLoggerPtr logger, ros::NodeHandle& root_nh, ros::NodeHandle& ctrl_nh)
{

  try
  {
    get_joint_names(&ctrl_nh,m_joint_names);
    if(m_joint_names.size()==1 && m_joint_names.front() == "all")
    {
      get_joint_names(&root_nh, m_joint_names);
    }

    if(m_joint_names.size()==0)
    {
      CNR_ERROR(logger, "Neither '"<< ctrl_nh.getNamespace() << "/controlled_joint(s)' nor '"
                          << ctrl_nh.getNamespace() << "'controlled_resources(s)' are specified. Abrot");
      CNR_RETURN_FALSE(logger);
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

    m_cfrmt = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");

    CNR_RETURN_TRUE(*logger);
  }
  catch(std::exception& e)
  {
    CNR_RETURN_FALSE(*logger, "Error caught: " + std::string(e.what()) );
  }
}

bool KinematicsStruct::saturateSpeed(Eigen::Ref<Eigen::VectorXd> qd_next,
                                     double max_velocity_multiplier,
                                     bool preserve_direction,
                                     std::stringstream* report)
{
  if(report)
  {
    *report << "[-----][SPEED SATURATION] INPUT  qd: " << TP(qd_next.transpose()) << "\n";
  }
  Eigen::VectorXd scale(nAx());
  for(size_t iAx=0;iAx<nAx();iAx++)
  {
    scale(iAx) = std::fabs(qd_next(iAx)) > speedLimit(iAx) * max_velocity_multiplier
               ? speedLimit(iAx) * max_velocity_multiplier/ std::fabs(qd_next(iAx) )
               : 1.0;
  }
  if(preserve_direction)
  {
    qd_next = scale.minCoeff() * qd_next;
  }
  else
  {
    qd_next = scale.asDiagonal() * qd_next;
  }

  if(report)
  {
    *report << (scale.minCoeff()<1 ? "[TRUE ]": "[FALSE]" )
            << "[SPEED SATURATION] OUTPUT qd: " << TP(qd_next.transpose()) << "\n";
  }

  return scale.minCoeff()<1;
}

bool KinematicsStruct::saturateSpeed(Eigen::Ref<Eigen::VectorXd> qd_next,
                                     const Eigen::Ref<const Eigen::VectorXd> qd_actual,
                                     double dt,
                                     double max_velocity_multiplier,
                                     bool preserve_direction,
                                     std::stringstream* report)
{
  bool saturated = saturateSpeed(qd_next, max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report<<"[-----][ACC   SATURATION] INPUT  qd: "<<TP(qd_next.transpose())
           <<"( qd actual:"<<TP(qd_actual.transpose())<<")\n";
  }
  Eigen::VectorXd qd_sup  = qd_actual + accelerationLimit() * dt;
  Eigen::VectorXd qd_inf  = qd_actual - accelerationLimit() * dt;
  Eigen::VectorXd dqd(nAx()); dqd.setZero();
  for(size_t iAx=0;iAx<nAx();iAx++)
  {
    dqd(iAx) = qd_next(iAx) > qd_sup(iAx) ? (qd_sup(iAx) - qd_next(iAx))
             : qd_next(iAx) < qd_inf(iAx) ? (qd_inf(iAx) - qd_next(iAx))
             : 0.0;
  }
  saturated |= dqd.cwiseAbs().maxCoeff()>0.0;
  if( preserve_direction )
  {
    Eigen::VectorXd dqd_dir = (qd_next - qd_actual).normalized();
    if(dqd.norm() < 1e-5)
    {
      dqd_dir.setZero();
    }

    if(dqd.minCoeff() * dqd.maxCoeff() >= 0.0)
    {
      qd_next = qd_next + (dqd.dot(dqd_dir) * dqd_dir);
    }
    else
    {
      *report << "Target vel     : " << TP(qd_next.transpose()) << "\n";
      *report << "Prev target vel: " << TP(qd_actual.transpose()) << "\n";
      *report << "qd_sup         : " << TP(qd_sup.transpose()) << "\n";
      *report << "qd_inf         : " << TP(qd_inf.transpose()) << "\n";
      *report << "Calc correction: " << TP(dqd.transpose()) << "\n";
      qd_next = qd_next + dqd;
    }
  }


  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            <<"[ACC   SATURATION] OUTPUT qd: "<<TP(qd_next.transpose())<< "\n";
  }
  return saturated;
}

bool KinematicsStruct::saturateSpeed(Eigen::Ref<Eigen::VectorXd> qd_next,
                                     const Eigen::Ref<const Eigen::VectorXd> qd_actual,
                                     const Eigen::Ref<const Eigen::VectorXd> q_actual,
                                     double dt,
                                     double max_velocity_multiplier,
                                     bool preserve_direction,
                                     std::stringstream* report)
{
  bool saturated = saturateSpeed(qd_next, qd_actual, dt,  max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report << "[-----][BRK   SATURATION] INPUT  qd: " << TP(qd_next.transpose()) 
            << " qd actual: " << TP(qd_actual.transpose()) << "\n";
  }
  Eigen::VectorXd braking_distance(this->nAx());
  for(size_t iAx=0; iAx<this->nAx();iAx++)
  {
    braking_distance(iAx)  = 0.5 * this->accelerationLimit(iAx)
                             * std::pow(std::abs(qd_next(iAx))/this->accelerationLimit(iAx) , 2.0);
  }

  Eigen::VectorXd q_saturated_qd = q_actual + qd_actual* dt;
  for(size_t iAx=0; iAx<this->nAx();iAx++)
  {
    if ((q_saturated_qd(iAx) > (this->upperLimit(iAx) - braking_distance(iAx))) && (qd_next(iAx)>0))
    {
      saturated |= true;
      qd_next(iAx) = std::max(0.0, qd_next(iAx) - this->accelerationLimit(iAx) * dt);
    }
    else if((q_saturated_qd(iAx)<(this->lowerLimit(iAx) + braking_distance(iAx))) && (qd_next(iAx)<0))
    {
      saturated |= true;
      qd_next(iAx) = std::min(0.0, qd_next(iAx) + this->accelerationLimit(iAx) * dt);
    }
  }

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            << "[BRK   SATURATION] OUTPUT qd: " << TP(qd_next.transpose()) << "\n";
  }
  return saturated;

}

bool KinematicsStruct::saturatePosition(Eigen::Ref<Eigen::VectorXd> q_next, std::stringstream* report)
{
  if(report)
  {
    *report << "[POS   SATURATION] INPUT  q: " << TP(q_next.transpose()) << "\n";
  }
  
  Eigen::VectorXd dq(q_next.rows());
  for(size_t iAx=0;iAx<nAx();iAx++)
  {
    dq(iAx)  = q_next(iAx) > upperLimit(iAx) ? (upperLimit(iAx) - q_next(iAx))
             : q_next(iAx) < lowerLimit(iAx) ? (lowerLimit(iAx) - q_next(iAx))
             : 0.0;
  }
  
  q_next += dq;
  
  if(report)
  {
    *report << (dq.cwiseAbs().maxCoeff()>0.0 ? "[TRUE ]": "[FALSE]" )
            << "[POS   SATURATION] OUTPUT q: " << TP(q_next.transpose()) << "\n";
  }

  return (dq.cwiseAbs().maxCoeff()>0.0);
}

KinematicStatus::KinematicStatus(KinematicsStructPtr kin) 
: kin_(kin) 
{   
  assert(kin); 
  this->q_     .resize(this->kin_->nAx());
  this->qd_    .resize(this->kin_->nAx());
  this->qdd_   .resize(this->kin_->nAx());
  this->effort_.resize(this->kin_->nAx());
  this->setZero();
} 

void KinematicStatus::setZero()
{
  this->q_     .setZero();
  this->qd_    .setZero();
  this->qdd_   .setZero();
  this->effort_.setZero();
  updateTransformation();
}

KinematicStatus::KinematicStatus(const KinematicStatus& cpy)
{
  this->kin_         = cpy.kin_   ;
  this->q_           = cpy.q_     ;
  this->qd_          = cpy.qd_    ;
  this->qdd_         = cpy.qdd_   ;
  this->effort_      = cpy.effort_;
  updateTransformation();
  
}

KinematicStatus& KinematicStatus::operator=(const KinematicStatus& rhs)
{
  this->kin_         = rhs.kin_   ;
  this->q_           = rhs.q_     ;
  this->qd_          = rhs.qd_    ;
  this->qdd_         = rhs.qdd_   ;
  this->effort_      = rhs.effort_;
  updateTransformation();
  return *this;
}


KinematicStatus& KinematicStatus::updateTransformation( )
{
  Tbt_      = kin_->getChain()->getTransformation(this->q());
  jacobian_ = kin_->getChain()->getJacobian(this->q());
  twist_    = kin_->getChain()->getTwistTool(this->q(), this->qd());
  twistd_   = kin_->getChain()->getDTwistTool(this->q(), this->qd(),this->qdd());
  
  return *this;
}


}



#undef TP
#undef SP
