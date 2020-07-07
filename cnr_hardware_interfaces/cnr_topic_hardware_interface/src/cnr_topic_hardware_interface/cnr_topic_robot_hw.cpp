#include <mutex>
#include <cnr_topic_hardware_interface/cnr_topic_robot_hw.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr_hardware_interface::TopicRobotHW, cnr_hardware_interface::RobotHW)



namespace cnr_hardware_interface
{


void setParam(TopicRobotHW* hw, const std::string& ns)
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
  
TopicRobotHW::TopicRobotHW()
{
  m_set_param = boost::bind(setParam,this,_1);
}
TopicRobotHW::~TopicRobotHW()
{
  m_mutex.lock();
  m_mutex.unlock();
  if(!m_shutted_down)
  {
    shutdown();
  }
}

bool TopicRobotHW::doInit( )
{
  CNR_TRACE_START(*m_logger);

  CNR_TRACE_START(*m_logger);
  if (!m_robot_hw_nh.getParam("joint_names", m_resource_names))
  {
    CNR_FATAL(*m_logger, m_robot_hw_nh.getNamespace()+"/joint_names' does not exist");
    CNR_FATAL(*m_logger, "ERROR DURING STARTING HARDWARE INTERFACE '" << m_robot_hw_nh.getNamespace() << "'");
    CNR_RETURN_FALSE(*m_logger);
  }
  CNR_DEBUG(*m_logger, "Create the TopicRobotHW (joint names: " << m_resource_names.size() << ")");

  std::string read_js_topic;
  if (!m_robot_hw_nh.getParam("feedback_joint_state_topic",read_js_topic))
  {
    add_diagnostic_message("ERROR", "feedback_joint_state_topic not defined", {{"Transition", "switching"}}, true);
    m_status = cnr_hardware_interface::ERROR;
    CNR_RETURN_FALSE(*m_logger);
  }
  
  std::string write_js_topic;
  if (!m_robot_hw_nh.getParam("command_joint_state_topic",write_js_topic))
  {
    add_diagnostic_message("ERROR", "command_joint_state_topic not defined", {{"Transition", "switching"}}, true);
    m_status = cnr_hardware_interface::ERROR;
    CNR_RETURN_FALSE(*m_logger);
  }
  
  int tmp;
  if (!m_robot_hw_nh.getParam("maximum_missing_cycles",tmp))
  {
    add_diagnostic_message("WARN", "maximum_missing_cycles not defined, set equal to 50", {{"Transition", "switching"}}, true);
    tmp=50;
  }
  m_max_missing_messages=tmp;
  
  m_pos.resize(m_resource_names.size());
  m_vel.resize(m_resource_names.size());
  m_eff.resize(m_resource_names.size());
  
  std::fill(m_pos.begin(),m_pos.end(),0.0);
  std::fill(m_vel.begin(),m_vel.end(),0.0);
  std::fill(m_eff.begin(),m_eff.end(),0.0);
  
  m_topic_received=false;
  
  m_js_sub = m_robot_hw_nh.subscribe<sensor_msgs::JointState>(read_js_topic,1,&cnr_hardware_interface::TopicRobotHW::jointStateCallback,this);  
  m_js_pub = m_robot_hw_nh.advertise<sensor_msgs::JointState>(write_js_topic,1);
    
  double timeout=10;
  if (!m_robot_hw_nh.getParam("feedback_joint_state_timeout",timeout))
  {
    add_diagnostic_message("WARN", "feedback_joint_state_timeout not defined, set equal to 10", {{"Transition", "switching"}}, true);
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
  
  m_msg.reset(new sensor_msgs::JointState());
  m_msg->name=m_resource_names;
  m_msg->position=m_pos;
  m_msg->velocity=m_vel;
  m_msg->effort.resize(m_resource_names.size());
  std::fill(m_msg->effort.begin(),m_msg->effort.end(),0.0);
  
  m_start_time=ros::Time::now();

  CNR_RETURN_TRUE(*m_logger);
}

void TopicRobotHW::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(m_mutex);
  std::vector<std::string> names= msg->name;
  std::vector<double>      pos  = msg->position;
  std::vector<double>      vel  = msg->velocity;
  std::vector<double>      eff  = msg->effort;
  
  if ( (pos.size()<m_resource_names.size()) || (vel.size()<m_resource_names.size()) || (eff.size()<m_resource_names.size()) || (names.size()<m_resource_names.size()))
  {
    std::string s = "Topic '"+ m_js_sub.getTopic() + " [Num publisher: "+std::to_string(m_js_pub.getNumSubscribers())+ "]' Mismatch in msg size: p:"+std::to_string((int)(pos.size())) + ", v:"  + std::to_string((int)(vel.size())) + ", e:"  + std::to_string((int)(eff.size()))+ ", names:"+ std::to_string((int)(m_resource_names.size()));
    CNR_ERROR_THROTTLE(*m_logger, 5.0, s );
    // add_diagnostic_message( "[jointStateCallback] Mismatch in msg size: p:"+std::to_string((int)(pos.size())) + ", v:"  + std::to_string((int)(vel.size())) + ", e:"  + std::to_string((int)(eff.size()))+ ", names:"+ std::to_string((int)(m_resource_names.size()))
    //                       , "callback", "ERROR", true );
    m_topic_received=false;
    return;
  }
  
  if (!name_sorting::permutationName(m_resource_names,names,pos,vel,eff, "ITIA TOPIC HW - jointStateCallback" ))
  {
    m_topic_received=false;
    CNR_WARN_THROTTLE(*m_logger, 0.1, m_robot_name << "Feedback joint states names are wrong!");
    return;
  }
  m_topic_received=true;

  for (unsigned int idx=0;idx<m_resource_names.size();idx++)
  {
    m_pos.at(idx)=pos.at(idx);
    m_vel.at(idx)=vel.at(idx);
    m_eff.at(idx)=eff.at(idx);
  }
}

bool TopicRobotHW::doRead(const ros::Time& time, const ros::Duration& period)
{

  if ((!m_topic_received) && ((time-m_start_time).toSec()>0.1))
  {
    m_missing_messages++;
  }
  else
  {
    m_missing_messages=0;
  }

  m_topic_received=false;
  if (m_missing_messages > m_max_missing_messages)
  {
    if( getStatus() == cnr_hardware_interface::RUNNING )
    {
      add_diagnostic_message("ERROR", "maximum_missing_cycles " + std::to_string( m_missing_messages ) + "s ", {{"read","missing messages"}}, true);
      return false;
    }
  }
  return true;
}

bool TopicRobotHW::doWrite(const ros::Time& time, const ros::Duration& period)
{

  CNR_TRACE_START_THROTTLE(*m_logger, 5.0 );
  if (!m_p_jh_active && !m_v_jh_active && !m_e_jh_active)
  {
    CNR_RETURN_TRUE_THROTTLE(*m_logger, 5.0 );
  }
  
  if (m_p_jh_active)
    m_msg->position = m_cmd_pos;
  else
  {
    m_msg->position.resize(m_resource_names.size());
    std::fill(m_msg->position.begin(),m_msg->position.end(),0.0);
  }
  
  if (m_v_jh_active)
    m_msg->velocity = m_cmd_vel;
  else
  {
    m_msg->velocity.resize(m_resource_names.size());
    std::fill(m_msg->velocity.begin(),m_msg->velocity.end(),0.0);
  }
  
  if (m_e_jh_active)
    m_msg->effort   = m_cmd_eff;
  else
  {
    m_msg->effort.resize(m_resource_names.size());
    std::fill(m_msg->effort.begin(),m_msg->effort.end(),0.0);
  }
  m_msg->name=m_resource_names;
  m_msg->header.stamp=ros::Time::now();

  m_js_pub.publish(m_msg);

  sensor_msgs::JointStatePtr msg(new sensor_msgs::JointState());
  m_msg.swap(msg);
  CNR_RETURN_TRUE_THROTTLE(*m_logger, 5.0 );
}

bool TopicRobotHW::doPrepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
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
  for (const hardware_interface::ControllerInfo& controller: start_list)
  {
    
    for (const hardware_interface::InterfaceResources& res: controller.claimed_resources)
    {
      if (!res.hardware_interface.compare("hardware_interface::PositionJointInterface"))
        p_jh_active=true;
      if (!res.hardware_interface.compare("hardware_interface::VelocityJointInterface"))
        v_jh_active=true;
      if (!res.hardware_interface.compare("hardware_interface::EffortJointInterface"))
        e_jh_active=true;
      if (!res.hardware_interface.compare("hardware_interface::VelEffJointInterface"))
      {
        v_jh_active=true;
        e_jh_active=true;
      }
      if (!res.hardware_interface.compare("hardware_interface::PosVelEffJointInterface"))
      {
        p_jh_active=true;
        v_jh_active=true;
        e_jh_active=true;
      }
    }
  }
  
  m_p_jh_active=p_jh_active;
  m_v_jh_active=v_jh_active;
  m_e_jh_active=e_jh_active;
  CNR_RETURN_TRUE(*m_logger);
}

bool TopicRobotHW::doShutdown()
{
  if(!m_shutted_down)
  {
    m_js_sub.shutdown();
  }
  return true;
}


}

