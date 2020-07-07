#include <pluginlib/class_list_macros.h>

#include <cnr_hardware_interface/cnr_robot_hw.h>
#include <cnr_fake_hardware_interface/cnr_fake_robot_hw.h>


PLUGINLIB_EXPORT_CLASS(cnr_hardware_interface::FakeRobotHW, cnr_hardware_interface::RobotHW)

namespace cnr_hardware_interface
{


void setParam(FakeRobotHW* hw, const std::string& ns)
{
  hw->m_robot_hw_nh.setParam("status/" + ns + "/feedback/name"    , hw->m_resource_names );
  hw->m_robot_hw_nh.setParam("status/" + ns + "/feedback/position", hw->m_pos );
  hw->m_robot_hw_nh.setParam("status/" + ns + "/feedback/velocity", hw->m_vel );
  hw->m_robot_hw_nh.setParam("status/" + ns + "/feedback/effort"  , hw->m_eff );
  hw->m_robot_hw_nh.setParam("status/" + ns + "/command/name"     , hw->m_resource_names );
  hw->m_robot_hw_nh.setParam("status/" + ns + "/command/position" , hw->m_cmd_pos );
  hw->m_robot_hw_nh.setParam("status/" + ns + "/command/velocity" , hw->m_cmd_vel );
  hw->m_robot_hw_nh.setParam("status/" + ns + "/command/effort"   , hw->m_cmd_eff );
}

FakeRobotHW::FakeRobotHW( )
  : m_msg( nullptr )
{

  m_set_param = boost::bind(setParam,this,_1);

}



FakeRobotHW::~FakeRobotHW()
{
  m_mutex.lock();
  m_mutex.unlock();
  if(!m_shutted_down)
  {
    shutdown();
  }
}

void FakeRobotHW::initialJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  m_msg  = new sensor_msgs::JointState();
  *m_msg = *msg;
}

bool FakeRobotHW::doInit( )
{
  CNR_TRACE_START(*m_logger);

  if (!m_robot_hw_nh.getParam("joint_names", m_resource_names))
  {
    CNR_FATAL(*m_logger, m_robot_hw_nh.getNamespace()+"/joint_names' does not exist");
    CNR_RETURN_FALSE(*m_logger, "doInit FAILED");
  }

  CNR_DEBUG(*m_logger, "Resources (" << m_resource_names.size() << "): " << cnr_controller_interface::to_string( m_resource_names ) );
  m_pos.resize(m_resource_names.size());
  m_vel.resize(m_resource_names.size());
  m_eff.resize(m_resource_names.size());
  
  std::fill(m_pos.begin(),m_pos.end(),0.0);
  std::fill(m_vel.begin(),m_vel.end(),0.0);
  std::fill(m_eff.begin(),m_eff.end(),0.0);
  
  if (m_robot_hw_nh.hasParam("initial_position"))
  {
    m_robot_hw_nh.getParam("initial_position",m_pos);
    std::string ss; for( auto const & p : m_pos) ss += std::to_string(p) + ", ";
    CNR_DEBUG(*m_logger, "Initial Position: <" << ss << ">");
  }
  else if(m_robot_hw_nh.hasParam("initial_position_from"))
  {
    std::string position_from;
    m_robot_hw_nh.getParam("position_from",position_from);

    CNR_DEBUG(*m_logger, "Position From: '" << position_from << "'");
    std::string position_ns = "/" + position_from + "/status/shutdown_configuration/position";
    if(!m_robot_hw_nh.hasParam(position_ns))
    {
      CNR_ERROR(*m_logger, "The param '"+position_ns+"' does not exit. pos superimposed to zero");
    }
    m_robot_hw_nh.getParam( position_ns, m_pos );
    std::string ss; for( auto const & p : m_pos) ss += std::to_string(p) + ", ";
    CNR_DEBUG(*m_logger, "Initial Position: <" << ss << ">");
  }

  double timeout=10;
  if (!m_robot_hw_nh.getParam("feedback_joint_state_timeout",timeout))
  {
    CNR_WARN(*m_logger, "The param '"<< m_robot_hw_nh.getNamespace() << "/feedback_joint_state_timeout' not defined, set equal to 10");
    timeout=10;
  }
  
  m_cmd_pos.resize(m_resource_names.size());
  m_cmd_vel.resize(m_resource_names.size());
  m_cmd_eff.resize(m_resource_names.size());
  
  m_cmd_pos=m_pos;
  m_cmd_vel=m_vel;
  m_cmd_pos=m_eff;
    
  for (const std::string& joint_name: m_resource_names)
  {
    
    auto i = &joint_name-&m_resource_names[0];
    
    hardware_interface::JointStateHandle state_handle(joint_name, 
                                                      &(m_pos.at(i)), 
                                                      &(m_vel.at(i)), 
                                                      &(m_eff.at(i)));

    m_js_jh.registerHandle(state_handle);
    
    m_p_jh.registerHandle( hardware_interface::JointHandle(state_handle, &(m_cmd_pos.at(i))) );
    m_v_jh.registerHandle( hardware_interface::JointHandle(state_handle, &(m_cmd_vel.at(i))) );
    m_e_jh.registerHandle( hardware_interface::JointHandle(state_handle, &(m_cmd_eff.at(i))) );
    
    m_pve_jh.registerHandle(hardware_interface::PosVelEffJointHandle(state_handle,&(m_cmd_pos.at(i)),&(m_cmd_vel.at(i)),&(m_cmd_eff.at(i))));
    m_ve_jh.registerHandle(hardware_interface::VelEffJointHandle(state_handle,&(m_cmd_vel.at(i)),&(m_cmd_eff.at(i))));
  }
    
  registerInterface(&m_js_jh);
  registerInterface(&m_p_jh);
  registerInterface(&m_v_jh);
  registerInterface(&m_e_jh);
  registerInterface(&m_pve_jh);
  registerInterface(&m_ve_jh);
  
  m_p_jh_active=m_v_jh_active=m_e_jh_active=false;
  CNR_RETURN_TRUE(*m_logger);
}


bool FakeRobotHW::doWrite(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE(*m_logger, 5.0 );
  //CNR_INFO_THROTTLE( *m_logger, 0.5 , "[   CMD] " << cnr_controller_interface::to_string( m_cmd_pos) );
  if (m_p_jh_active)
  {
    m_pos = m_cmd_pos;
  }
  else
  {
    std::fill(m_pos.begin(),m_pos.end(),0.0);
  }
  
  if (m_v_jh_active)
  {
    m_vel = m_cmd_vel;
  }
  else
  {
    m_vel.resize(m_resource_names.size());
    std::fill(m_vel.begin(),m_vel.end(),0.0);
  }
  
  if (m_e_jh_active)
  {
    m_eff   = m_cmd_eff;
  }
  else
  {
    m_eff.resize(m_resource_names.size());
    std::fill(m_eff.begin(),m_eff.end(),0.0);
  }
  CNR_RETURN_TRUE_THROTTLE(*m_logger, 5.0 );
}

bool FakeRobotHW::doPrepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  CNR_TRACE_START(*m_logger);
  bool p_jh_active, v_jh_active, e_jh_active;

  p_jh_active=m_p_jh_active;
  v_jh_active=m_v_jh_active;
  e_jh_active=m_e_jh_active;

  for (const hardware_interface::ControllerInfo& controller: stop_list)
  {
    for (const hardware_interface::InterfaceResources& res: controller.claimed_resources)
    {
      if (!res.hardware_interface.compare("hardware_interface::PositionJointInterface"))
        p_jh_active=false;
      if (!res.hardware_interface.compare("hardware_interface::VelocityJointInterface"))
        v_jh_active=false;
      if (!res.hardware_interface.compare("hardware_interface::EffortJointInterface"))
        e_jh_active=false;
      if (!res.hardware_interface.compare("hardware_interface::VelEffJointInterface"))
      {
        v_jh_active=false;
        e_jh_active=false;
      }
      if (!res.hardware_interface.compare("hardware_interface::PosVelEffJointInterface"))
      {
        p_jh_active=false;
        v_jh_active=false;
        e_jh_active=false;
      }
    }
  }

  std::vector<std::string> resources;
  for (const hardware_interface::ControllerInfo& controller: start_list)
  {
    for (const hardware_interface::InterfaceResources& res: controller.claimed_resources)
    {
      resources.push_back(res.hardware_interface);
      p_jh_active =  ( res.hardware_interface == "hardware_interface::PositionJointInterface"  )
                  || ( res.hardware_interface == "hardware_interface::PosVelEffJointInterface" )
                  ? true : p_jh_active;

      v_jh_active =  ( res.hardware_interface == "hardware_interface::VelocityJointInterface"  )
                  || ( res.hardware_interface == "hardware_interface::VelEffJointInterface"    )
                  || ( res.hardware_interface == "hardware_interface::PosVelEffJointInterface" )
                  ? true : v_jh_active;

      e_jh_active =  ( res.hardware_interface == "hardware_interface::EffortJointInterface"    )
                  || ( res.hardware_interface == "hardware_interface::VelEffJointInterface"    )
                  || ( res.hardware_interface == "hardware_interface::PosVelEffJointInterface" )
                  ? true : e_jh_active;
    }
  }
  m_p_jh_active = p_jh_active;
  m_v_jh_active = v_jh_active;
  m_e_jh_active = e_jh_active;
  CNR_DEBUG(*m_logger, " Pos joint handle active? " << m_p_jh_active );
  CNR_DEBUG(*m_logger, " Vel joint handle active? " << m_v_jh_active );
  CNR_DEBUG(*m_logger, " Eff joint handle active? " << m_e_jh_active );
  CNR_RETURN_TRUE(*m_logger, "Active hardware interfaces: " + cnr_controller_interface::to_string(resources) );

}


bool FakeRobotHW::doCheckForConflict(const std::list< hardware_interface::ControllerInfo >& info)
{
  CNR_TRACE_START(*m_logger);
  // Each controller can use more than a hardware_interface for a single joint (for example: position, velocity, effort). 
  // One controller can control more than one joint.
  // A joint can be used only by a controller.
  
  std::vector<bool> global_joint_used(m_resource_names.size());
  std::fill(global_joint_used.begin(),global_joint_used.end(),false);
  
  for (hardware_interface::ControllerInfo controller: info)
  {
    std::vector<bool> single_controller_joint_used(m_resource_names.size());
    std::fill(single_controller_joint_used.begin(),single_controller_joint_used.end(),false);
    
    for (hardware_interface::InterfaceResources res: controller.claimed_resources)
    {
      for (std::string name: res.resources)
      {
        for (unsigned int iJ=0;iJ<m_resource_names.size();iJ++)
        {
          if (!name.compare(m_resource_names.at(iJ)))
          {
            if (global_joint_used.at(iJ)) // if already used by another
            {
              ROS_ERROR("Joint %s is already used by another controller",name.c_str());
              diagnostic_msgs::DiagnosticStatus diag;
              diag.name=m_robot_hw_nh.getNamespace();
              diag.hardware_id=m_robot_hw_nh.getNamespace();
              diag.level=diagnostic_msgs::DiagnosticStatus::ERROR;
              diag.message="Hardware interface "+m_robot_hw_nh.getNamespace()+" run time: Joint " + name + " is already used by another controller";

              std::lock_guard<std::mutex> lock(m_mutex);
              m_diagnostic.status.push_back(diag);
              
              CNR_RETURN_TRUE(*m_logger, diag.message);
            }
            else
            {
              single_controller_joint_used.at(iJ) = true;
            }
          }
        }
      }
    }
    for (unsigned int iJ=0;iJ<m_resource_names.size();iJ++)
    {
      global_joint_used.at(iJ)= global_joint_used.at(iJ) || single_controller_joint_used.at(iJ);
    }
    
  }
  CNR_RETURN_FALSE(*m_logger);
}



}
