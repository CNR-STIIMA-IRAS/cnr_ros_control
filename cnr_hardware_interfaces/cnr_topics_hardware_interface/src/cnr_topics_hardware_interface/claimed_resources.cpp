#include <ros/param.h>
#include <cnr_topics_hardware_interface/claimed_resources.h>
#include <name_sorting/name_sorting.h>

namespace cnr_hardware_interface
{



/**
 *
 *
 *
 *
 *
 *
 *
 *
 */

JointClaimedResource::JointClaimedResource(const cnr_hardware_interface::JointResource& jr,
                                           ros::NodeHandle& robothw_nh, 
                                           std::map<std::string, bool> & topics_received)
: cnr_hardware_interface::ClaimedResource<sensor_msgs::JointState> (jr, jr.m_joint_names, robothw_nh, topics_received), 
m_p_jh_active(false),
m_v_jh_active(false),
m_e_jh_active(false)
{
  m_pos.resize(m_resource_names.size(), 0);
  m_vel.resize(m_resource_names.size(), 0);
  m_eff.resize(m_resource_names.size(), 0);

  m_cmd_pos.resize(m_resource_names.size(), 0);
  m_cmd_vel.resize(m_resource_names.size(), 0);
  m_cmd_eff.resize(m_resource_names.size(), 0);

  m_cmd_pos = m_pos;
  m_cmd_vel = m_vel;
  m_cmd_pos = m_eff;

}


void JointClaimedResource::callback(const sensor_msgs::JointStateConstPtr& msg, const std::string& topic)
{
  ClaimedResource::callback(msg, topic);

  std::lock_guard<std::mutex> lock(m_mutex);
  std::vector<std::string> names = msg->name;
  std::vector<double> pos = msg->position;
  std::vector<double> vel = msg->velocity;
  std::vector<double> eff = msg->effort;

  std::vector<unsigned int> ax_indexes = m_idxes_ax_map[ topic ];
  if ((pos.size() < ax_indexes.size()) || (vel.size() < ax_indexes.size()) || (eff.size() < ax_indexes.size()) || (names.size() < ax_indexes.size()))
  {
    ROS_FATAL("Dimension are wrong: pos %zu, vel %zu, eff %zu, names %zu, expected %zu", pos.size(), vel.size(), eff.size(), names.size(), ax_indexes.size());
    m_topics_received[topic] = false;
    return;
  }

  std::stringstream report;
  if (!name_sorting::permutationName(m_resource_names_map.at(topic), names, pos, vel, eff, &report))
  {
    m_topics_received[topic] = false;
    ROS_WARN_THROTTLE(0.1, "[%s] feedback joint states names are wrong!", m_namespace.c_str());
    return;
  }
  m_topics_received[topic] = true;
  for (unsigned int idx = 0; idx < m_resource_names_map.at(topic).size(); idx++)
  {
    m_pos.at(ax_indexes.at(idx)) = pos.at(idx);
    m_vel.at(ax_indexes.at(idx)) = vel.at(idx);
    m_eff.at(ax_indexes.at(idx)) = eff.at(idx);
  }
}


void JointClaimedResource::init()
{
  cnr_hardware_interface::ClaimedResource<sensor_msgs::JointState>::init();
  for (std::string& joint_name : m_resource_names)
  {
    auto i = &joint_name - &(m_resource_names[0]);

    hardware_interface::JointStateHandle state_handle(joint_name,
        &(m_pos.at(i)),
        &(m_vel.at(i)),
        &(m_eff.at(i)));


    m_js_jh.registerHandle(state_handle);

    m_p_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_pos.at(i))));
    m_v_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_vel.at(i))));
    m_e_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_eff.at(i))));

    m_pve_jh.registerHandle(hardware_interface::PosVelEffJointHandle(state_handle, &(m_cmd_pos.at(i)), &(m_cmd_vel.at(i)), &(m_cmd_eff.at(i))));
    m_ve_jh.registerHandle(hardware_interface::VelEffJointHandle(state_handle, &(m_cmd_vel.at(i)), &(m_cmd_eff.at(i))));
  }

  m_p_jh_active = m_v_jh_active = m_e_jh_active = false;

  if (m_pub_msg.size())
  {
    m_pub_msg.front().reset(new sensor_msgs::JointState());
    m_pub_msg.front()->name         = m_resource_names;
    m_pub_msg.front()->position     = m_pos;
    m_pub_msg.front()->velocity     = m_vel;
    m_pub_msg.front()->effort       = m_eff;
    std::fill(m_pub_msg.front()->effort.begin(), m_pub_msg.front()->effort.end(), 0.0);
  }
}


void JointClaimedResource::shutdown()
{
  if (!m_shutted_down)
  {
    cnr_hardware_interface::ClaimedResource<sensor_msgs::JointState>::shutdown();
  }
}


void JointClaimedResource::setParam(const std::string& ns)
{
  ros::param::set(m_namespace + "/status/joint_resource/" + ns + "/feedback/name", m_resource_names);
  ros::param::set(m_namespace + "/status/joint_resource/" + ns + "/feedback/position", m_pos);
  ros::param::set(m_namespace + "/status/joint_resource/" + ns + "/feedback/velocity", m_vel);
  ros::param::set(m_namespace + "/status/joint_resource/" + ns + "/feedback/effort", m_eff);
  ros::param::set(m_namespace + "/status/joint_resource/" + ns + "/command/name", m_resource_names);
  ros::param::set(m_namespace + "/status/joint_resource/" + ns + "/command/position", m_cmd_pos);
  ros::param::set(m_namespace + "/status/joint_resource/" + ns + "/command/velocity", m_cmd_vel);
  ros::param::set(m_namespace + "/status/joint_resource/" + ns + "/command/effort", m_cmd_eff);
}


void JointClaimedResource::write(const ros::Time& time, const ros::Duration& period)
{
  if (m_pub.size() > 0 && m_pub.front())
  {
    if (!m_p_jh_active && !m_v_jh_active && !m_e_jh_active)
    {
      return;
    }

    if (m_p_jh_active)
    {
      m_pub_msg.front()->position = m_cmd_pos;
    }
    else
    {
      m_pub_msg.front()->position.resize(m_cmd_pos.size());
      std::fill(m_pub_msg.front()->position.begin(), m_pub_msg.front()->position.end(), 0.0);
    }

    if (m_v_jh_active)
      m_pub_msg.front()->velocity = m_cmd_vel;
    else
    {
      m_pub_msg.front()->velocity.resize(m_cmd_vel.size());
      std::fill(m_pub_msg.front()->velocity.begin(), m_pub_msg.front()->velocity.end(), 0.0);
    }

    if (m_e_jh_active)
      m_pub_msg.front()->effort   = m_cmd_eff;
    else
    {
      m_pub_msg.front()->effort.resize(m_cmd_eff.size());
      std::fill(m_pub_msg.front()->effort.begin(), m_pub_msg.front()->effort.end(), 0.0);
    }
    m_pub_msg.front()->name = m_resource_names;
    m_pub_msg.front()->header.stamp = ros::Time::now();

    ClaimedResource::write(time, period);
  }
}


bool JointClaimedResource::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  bool p_jh_active, v_jh_active, e_jh_active;
  p_jh_active = v_jh_active = e_jh_active = false;

  for (const hardware_interface::ControllerInfo& controller : start_list)
  {
    for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
    {
      if (!res.hardware_interface.compare("hardware_interface::PositionJointInterface"))
        p_jh_active = true;
      if (!res.hardware_interface.compare("hardware_interface::VelocityJointInterface"))
        v_jh_active = true;
      if (!res.hardware_interface.compare("hardware_interface::EffortJointInterface"))
        e_jh_active = true;
      if (!res.hardware_interface.compare("hardware_interface::VelEffJointInterface"))
      {
        v_jh_active = true;
        e_jh_active = true;
      }
      if (!res.hardware_interface.compare("hardware_interface::PosVelEffJointInterface"))
      {
        p_jh_active = true;
        v_jh_active = true;
        e_jh_active = true;
      }
    }
  }

  m_p_jh_active = p_jh_active;
  m_v_jh_active = v_jh_active;
  m_e_jh_active = e_jh_active;
  return true;
}


/**
 *
 *
 *
 *
 *
 *
 *
 *
 */

AnalogClaimedResource::AnalogClaimedResource(const cnr_hardware_interface::AnalogResource& ar, 
                                             ros::NodeHandle& robothw_nh, 
                                             std::map< std::string, bool> & topics_received)
: cnr_hardware_interface::ClaimedResource<std_msgs::Float64MultiArray>(ar, ar.m_channel_names, robothw_nh, topics_received)
, m_a_h_active( false )
, m_a_sh_active( false )
{
  m_state.resize(ar.m_channel_names.size(), 0);
  m_output.resize(ar.m_channel_names.size(), 0);

}

void AnalogClaimedResource::callback(const std_msgs::Float64MultiArray::ConstPtr& msg, const std::string& topic)
{
  ClaimedResource::callback(msg, topic);

  std::lock_guard<std::mutex> lock(m_mutex);
  if (msg->data.size() != m_state.size())
  {
    ROS_FATAL("Dimension are wrong: msg %zu, state %zu", msg->data.size(), m_state.size());
    m_topics_received[topic] = false;
    return;
  }
  std::vector<std::string> names(m_state.size());
  for (size_t i = 0; i < m_state.size(); i++)
  {
    names[i] = msg->layout.dim[i].label;
  }

  std::vector<double>       values = msg->data;
  std::vector<unsigned int> ax_indexes = m_idxes_ax_map[ topic ];
  if (values.size() < ax_indexes.size())
  {
    ROS_FATAL("Dimension are wrong: val %zu, expected %zu", values.size(), ax_indexes.size());
    m_topics_received[topic] = false;
    return;
  }

  std::stringstream report;
  if (!name_sorting::permutationName(m_resource_names_map.at(topic), names, values, &report))
  {
    m_topics_received[topic] = false;
    ROS_WARN_THROTTLE(0.1, "[%s] Error", report.str().c_str());
    ROS_WARN_THROTTLE(0.1, "[%s] feedback joint states names are wrong!", m_namespace.c_str());
    return;
  }
  m_topics_received[topic] = true;
  for (unsigned int idx = 0; idx < m_resource_names_map.at(topic).size(); idx++)
  {
    m_state.at(ax_indexes.at(idx)) = values.at(idx);
  }
}


void AnalogClaimedResource::init()
{
  cnr_hardware_interface::ClaimedResource<std_msgs::Float64MultiArray>::init();

  for (std::string& channel_name : m_resource_names)
  {
    auto i = &channel_name - &(m_resource_names[0]);

    hardware_interface::AnalogStateHandle state_handle(channel_name, &(m_state.at(i)));

    m_a_sh.registerHandle(state_handle);
    m_a_h.registerHandle(hardware_interface::AnalogHandle(state_handle, &(m_output.at(i))));
  }

  m_a_sh_active = m_a_h_active = false;

  if (m_pub_msg.size())
  {
    m_pub_msg.front().reset(new std_msgs::Float64MultiArray());
    m_pub_msg.front()->data.resize(m_resource_names.size(), 0);
  }
}



void AnalogClaimedResource::shutdown()
{
  cnr_hardware_interface::ClaimedResource<std_msgs::Float64MultiArray>::shutdown();
}


void AnalogClaimedResource::setParam(const std::string& ns)
{
  ros::param::set(m_namespace + "/status/analog_resource/" + ns + "/feedback/name", m_resource_names);
  ros::param::set(m_namespace + "/status/analog_resource/" + ns + "/feedback/values", m_state);
  ros::param::set(m_namespace + "/status/analog_resource/" + ns + "/command/name", m_resource_names);
  ros::param::set(m_namespace + "/status/analog_resource/" + ns + "/command/values", m_output);
}


void AnalogClaimedResource::write(const ros::Time& time, const ros::Duration& period)
{
  if (m_pub.size() > 0 && m_pub.front())
  {
    m_pub_msg.front()->data = m_output;

    ClaimedResource::write(time, period);
  }
}


bool AnalogClaimedResource::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
                                          const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  bool a_h_active = m_a_h_active;
  bool a_sh_active = m_a_sh_active;
  for (const hardware_interface::ControllerInfo& controller : start_list)
  {
    for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
    {
      a_sh_active = !res.hardware_interface.compare("hardware_interface::AnalogStateInterface");
      a_h_active = !res.hardware_interface.compare("hardware_interface::AnalogInterface");
    }
  }
  m_a_h_active = a_h_active;
  m_a_sh_active = a_sh_active;
  return true;
}


/**
 *
 *
 *
 *
 *
 *
 *
 *
 */

ForceTorqueClaimedResource::ForceTorqueClaimedResource(const cnr_hardware_interface::ForceTorqueResource& wr,
                                                       ros::NodeHandle& robothw_nh,
                                                       std::map< std::string, bool> & topics_received)
: cnr_hardware_interface::ClaimedResource<geometry_msgs::WrenchStamped> (wr, {wr.m_sensor_name}, robothw_nh, topics_received)
, m_w_h_active(false)
, m_w_sh_active(false)

{
  assert(wr.m_subscribed_topics.size() == 1);
  size_t l = __LINE__;
  try
  {
    l = __LINE__;
    m_frame_id = wr.m_frame_id;
    m_state.resize(6, 0);
    l = __LINE__;
    m_output.resize(6, 0);
    l = __LINE__;
  }
  catch (std::exception& e)
  {
    ROS_FATAL("ClaimedResource error %s (at line:%zu)", e.what(), l);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    throw std::runtime_error(("ClaimedResource error"  + std::string(e.what())) .c_str());
  }
  catch (...)
  {
    ROS_FATAL("ClaimedResource error. Unhandled expcetion at line:%zu", l);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    throw std::runtime_error("ClaimedResource error");
  }


}


void ForceTorqueClaimedResource::callback(const geometry_msgs::WrenchStamped::ConstPtr& msg, const std::string& topic)
{
  ClaimedResource::callback(msg, topic);

  std::lock_guard<std::mutex> lock(m_mutex);

  m_topics_received[topic] = true;
  m_state.at(0) = msg->wrench.force.x;
  m_state.at(1) = msg->wrench.force.y;
  m_state.at(2) = msg->wrench.force.z;
  m_state.at(3) = msg->wrench.torque.x;
  m_state.at(4) = msg->wrench.torque.y;
  m_state.at(5) = msg->wrench.torque.z;
}


void ForceTorqueClaimedResource::init()
{
  cnr_hardware_interface::ClaimedResource<geometry_msgs::WrenchStamped>::init();

  hardware_interface::ForceTorqueStateHandle state_handle(m_resource_names.front()
      , m_frame_id
      , &(m_state.at(0))
      , &(m_state.at(3)));
  hardware_interface::ForceTorqueSensorHandle sensor_handle(m_resource_names.front()
      , m_frame_id
      , &(m_state.at(0))
      , &(m_state.at(3)));

  m_w_sensor_h.registerHandle(sensor_handle);
  m_w_sh.registerHandle(state_handle);
  m_w_h.registerHandle(hardware_interface::ForceTorqueHandle(state_handle, &(m_output.at(0)), &(m_output.at(3))));


  if (m_pub_msg.size())
  {
    m_w_sh_active = m_w_h_active = false;
    m_pub_msg.front().reset(new geometry_msgs::WrenchStamped());
  }
}



void ForceTorqueClaimedResource::shutdown()
{
  if (!m_shutted_down)
  {
    cnr_hardware_interface::ClaimedResource<geometry_msgs::WrenchStamped>::shutdown();
  }
}


void ForceTorqueClaimedResource::setParam(const std::string& ns)
{
  ros::param::set(m_namespace + "/status/analog_resource/" + ns + "/feedback/name", m_resource_names);
  ros::param::set(m_namespace + "/status/analog_resource/" + ns + "/feedback/values", m_state);
  ros::param::set(m_namespace + "/status/analog_resource/" + ns + "/command/name", m_resource_names);
  ros::param::set(m_namespace + "/status/analog_resource/" + ns + "/command/values", m_output);
}


void ForceTorqueClaimedResource::write(const ros::Time& time, const ros::Duration& period)
{
  if (m_pub.size() > 0 && m_pub.front())
  {
    m_pub_msg.front()->wrench.force.x  = m_output.at(0);
    m_pub_msg.front()->wrench.force.y  = m_output.at(1);
    m_pub_msg.front()->wrench.force.z  = m_output.at(2);
    m_pub_msg.front()->wrench.torque.x = m_output.at(3);
    m_pub_msg.front()->wrench.torque.y = m_output.at(4);
    m_pub_msg.front()->wrench.torque.z = m_output.at(5);

    ClaimedResource::write(time, period);
  }
}


bool ForceTorqueClaimedResource::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  bool h_active = m_w_h_active;
  bool sh_active = m_w_sh_active;
  for (const hardware_interface::ControllerInfo& controller : start_list)
  {
    for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
    {
      sh_active = !res.hardware_interface.compare("hardware_interface::ForceTorqueStateHandle");
      h_active  = !res.hardware_interface.compare("hardware_interface::ForceTorqueInterface");
    }
  }
  m_w_h_active = h_active;
  m_w_sh_active = sh_active;
  return true;
}





/**
 *
 *
 *
 *
 *
 *
 *
 *
 */

PoseClaimedResource::PoseClaimedResource(const cnr_hardware_interface::PoseResource& pr, ros::NodeHandle& robothw_nh, std::map< std::string, bool> & topics_received)
: cnr_hardware_interface::ClaimedResource< geometry_msgs::PoseStamped > (pr, {pr.m_frame_id}, robothw_nh, topics_received)
, m_p_h_active(false)
, m_p_sh_active(false)
{
  size_t l = __LINE__;
  try
  {
    l = __LINE__;
    m_state = geometry_msgs::Pose();
    l = __LINE__;
    m_output = geometry_msgs::Pose();
    l = __LINE__;
  }
  catch (std::exception& e)
  {
    ROS_FATAL("ClaimedResource error %s (at line:%zu)", e.what(), l);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    throw std::runtime_error(("ClaimedResource error"  + std::string(e.what())) .c_str());
  }
  catch (...)
  {
    ROS_FATAL("ClaimedResource error. Unhandled expcetion at line:%zu", l);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    throw std::runtime_error("ClaimedResource error");
  }


}


void PoseClaimedResource::callback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& topic)
{
  ClaimedResource::callback(msg, topic);

  std::lock_guard<std::mutex> lock(m_mutex);

  m_topics_received[topic] = true;
  m_state = msg->pose;
}


void PoseClaimedResource::init()
{
  cnr_hardware_interface::ClaimedResource<geometry_msgs::PoseStamped>::init();

  hardware_interface::PoseStateHandle state_handle(m_resource_names.front()
      , &(m_state));

  m_p_sh.registerHandle(state_handle);
  m_p_h.registerHandle(hardware_interface::PoseHandle(state_handle, &(m_output)));

  m_p_sh_active = m_p_h_active = false;

  if (m_pub_msg.size())
  {
    m_pub_msg.front().reset(new geometry_msgs::PoseStamped());
  }
}



void PoseClaimedResource::shutdown()
{
  if (!m_shutted_down)
  {
    cnr_hardware_interface::ClaimedResource<geometry_msgs::PoseStamped>::shutdown();
  }
}


void PoseClaimedResource::setParam(const std::string& ns)
{
  ros::param::set(m_namespace + "/status/pose_resource/" + ns + "/feedback/name", m_resource_names);
//ros::param::set(m_namespace + "/status/pose_resource/"+ns+"/feedback/values"  , m_state          );
  ros::param::set(m_namespace + "/status/pose_resource/" + ns + "/command/name", m_resource_names);
//ros::param::set(m_namespace + "/status/pose_resource/"+ns+"/command/values"   , m_output         );
}



void PoseClaimedResource::write(const ros::Time& time, const ros::Duration& period)
{
  if (m_pub.size() > 0 && m_pub.front())
  {
    m_pub_msg.front()->header.stamp = ros::Time::now();
    m_pub_msg.front()->pose = m_output;

    ClaimedResource::write(time, period);
  }
}


bool PoseClaimedResource::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  bool h_active = m_p_h_active;
  bool sh_active = m_p_sh_active;
  for (const hardware_interface::ControllerInfo& controller : start_list)
  {
    for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
    {
      sh_active = !res.hardware_interface.compare("hardware_interface::PoseStateHandle");
      h_active  = !res.hardware_interface.compare("hardware_interface::PoseCommandInterface");
    }
  }
  m_p_h_active = h_active;
  m_p_sh_active = sh_active;
  return true;
}









/**
 *
 *
 *
 *
 *
 *
 *
 *
 */

TwistClaimedResource::TwistClaimedResource(const cnr_hardware_interface::TwistResource& pr, ros::NodeHandle& robothw_nh, std::map< std::string, bool> & topics_received)
  : cnr_hardware_interface::ClaimedResource< geometry_msgs::TwistStamped > (pr, pr.m_frames_id, robothw_nh, topics_received)
#define ll l = __LINE__;
{
  size_t l = __LINE__;
  try
  {
    assert(pr.m_subscribed_topics.size() == pr.m_frames_id.size());
    m_state.resize(pr.m_subscribed_topics.size(), geometry_msgs::TwistStamped());
    ll;
    for (size_t i = 0; i < m_state.size(); i++)
    {
      m_state.at(i).header.frame_id = pr.m_frames_id.at(i);
    }

    m_output.resize(pr.m_published_topics.size(), geometry_msgs::TwistStamped());
    ll;
    for (size_t i = 0; i < m_output.size(); i++)
    {
      m_output.at(i).header.frame_id = pr.m_frames_id.at(i);
    }

    m_t_sh.resize(m_state.size());    //interface for reading joint state
    m_t_h.resize(m_output.size());     //interface for reading joint state

    m_t_sh_active.resize(m_state.size(), false);
    m_t_h_active .resize(m_output.size(), false);
  }
  catch (std::exception& e)
  {
    ROS_FATAL("ClaimedResource error %s (at line:%zu)", e.what(), l);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    throw std::runtime_error(("ClaimedResource error"  + std::string(e.what())) .c_str());
  }
  catch (...)
  {
    ROS_FATAL("ClaimedResource error. Unhandled expcetion at line:%zu", l);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    throw std::runtime_error("ClaimedResource error");
  }
}


void TwistClaimedResource::callback(const geometry_msgs::TwistStamped::ConstPtr& msg, const std::string& topic)
{
  ClaimedResource::callback(msg, topic);

  std::lock_guard<std::mutex> lock(m_mutex);

  m_topics_received[topic] = true;
  for (geometry_msgs::TwistStamped & state : m_state)
  {
    if (state.header.frame_id == msg->header.frame_id)
    {
      state.twist = msg->twist;
      state.header.stamp = ros::Time::now();
    }
  }
}


void TwistClaimedResource::init()
{
  cnr_hardware_interface::ClaimedResource<geometry_msgs::TwistStamped>::init();
  for (size_t i = 0; i < m_state.size(); i++)
  {
    hardware_interface::TwistStateHandle state_handle(m_resource_names.at(i)
        , &(m_state.at(i).twist));

    m_t_sh.at(i).registerHandle(state_handle);
    m_t_h.at(i).registerHandle(hardware_interface::TwistHandle(state_handle, &(m_output.at(i).twist)));

    m_t_sh_active.at(i) = m_t_h_active.at(i) = false;
  }

  for (size_t i = 0; i < m_pub_msg.size(); i++)
  {
    m_pub_msg.at(i).reset(new geometry_msgs::TwistStamped());
  }
}



void TwistClaimedResource::shutdown()
{
  if (!m_shutted_down)
  {
    cnr_hardware_interface::ClaimedResource<geometry_msgs::TwistStamped>::shutdown();
  }
}


void TwistClaimedResource::setParam(const std::string& ns)
{
  ros::param::set(m_namespace + "/status/twist_resource/" + ns + "/feedback/name", m_resource_names);
//ros::param::set(m_namespace + "/status/twist_resource/"+ns+"/feedback/values"  , m_state          );
  ros::param::set(m_namespace + "/status/twist_resource/" + ns + "/command/name", m_resource_names);
//ros::param::set(m_namespace + "/status/twist_resource/"+ns+"/command/values"   , m_output         );
}



void TwistClaimedResource::write(const ros::Time& time, const ros::Duration& period)
{
  if (m_pub.size() > 0)
  {
    for (size_t i = 0; i < m_pub.size(); i++)
    {
      m_pub_msg.at(i)->header.stamp     = ros::Time::now();
      m_pub_msg.at(i)->header.frame_id  = m_output.at(i).header.frame_id;
      m_pub_msg.at(i)->twist            = m_output.at(i).twist;
    }

    ClaimedResource::write(time, period);
  }
}


bool TwistClaimedResource::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  for (size_t i = 0; i < m_resource_names.size(); i++)
  {
    bool h_active = m_t_h_active.at(i);
    bool sh_active = m_t_sh_active.at(i);
    for (const hardware_interface::ControllerInfo& controller : start_list)
    {
      for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
      {
        sh_active = !res.hardware_interface.compare("hardware_interface::TwistStateHandle");
        h_active  = !res.hardware_interface.compare("hardware_interface::TwistCommandInterface");
      }
    }
    m_t_h_active.at(i) = h_active;
    m_t_sh_active.at(i) = sh_active;
  }
  return true;
}


}
