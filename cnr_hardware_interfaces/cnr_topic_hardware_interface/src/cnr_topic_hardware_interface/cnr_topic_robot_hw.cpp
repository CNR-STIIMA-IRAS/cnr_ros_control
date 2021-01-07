/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <sstream>
#include <mutex>
#include <cnr_topic_hardware_interface/cnr_topic_robot_hw.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr_hardware_interface::TopicRobotHW, cnr_hardware_interface::RobotHW)

namespace cnr_hardware_interface
{


void setParam(TopicRobotHW* hw, const std::string& ns)
{
  hw->m_robothw_nh.setParam("status/" + ns + "/feedback/name", hw->m_resource_names);
  hw->m_robothw_nh.setParam("status/" + ns + "/feedback/position", hw->m_pos);
  hw->m_robothw_nh.setParam("status/" + ns + "/feedback/velocity", hw->m_vel);
  hw->m_robothw_nh.setParam("status/" + ns + "/feedback/effort", hw->m_eff);
  hw->m_robothw_nh.setParam("status/" + ns + "/command/name", hw->m_resource_names);
  hw->m_robothw_nh.setParam("status/" + ns + "/command/position", hw->m_cmd_pos);
  hw->m_robothw_nh.setParam("status/" + ns + "/command/velocity", hw->m_cmd_vel);
  hw->m_robothw_nh.setParam("status/" + ns + "/command/effort", hw->m_cmd_eff);
}

TopicRobotHW::TopicRobotHW()
{
  m_set_status_param = boost::bind(setParam, this, _1);
}
TopicRobotHW::~TopicRobotHW()
{
  m_mutex.lock();
  m_mutex.unlock();
  if (!m_shutted_down)
  {
    shutdown();
  }
}

bool TopicRobotHW::doInit()
{
  std::stringstream report;

  CNR_TRACE_START(m_logger);
  if (!m_robothw_nh.getParam("joint_names", m_resource_names))
  {
    CNR_FATAL(m_logger, m_robothw_nh.getNamespace() + "/joint_names' does not exist");
    CNR_FATAL(m_logger, "ERROR DURING STARTING HARDWARE INTERFACE '" << m_robothw_nh.getNamespace() << "'");
    CNR_RETURN_FALSE(m_logger);
  }
  CNR_DEBUG(m_logger, "Create the TopicRobotHW (joint names: " << m_resource_names.size() << ")");

  std::string read_js_topic;
  if (!m_robothw_nh.getParam("feedback_joint_state_topic", read_js_topic))
  {
    addDiagnosticsMessage("ERROR", "feedback_joint_state_topic not defined", {{"Transition", "switching"}}, &report);
    CNR_ERROR(m_logger, report.str() );

    m_status = cnr_hardware_interface::ERROR;
    CNR_RETURN_FALSE(m_logger);
  }

  std::string write_js_topic;
  if (!m_robothw_nh.getParam("command_joint_state_topic", write_js_topic))
  {
    addDiagnosticsMessage("ERROR", "command_joint_state_topic not defined", {{"Transition", "switching"}}, &report);
    CNR_ERROR(m_logger, report.str() );

    m_status = cnr_hardware_interface::ERROR;
    CNR_RETURN_FALSE(m_logger);
  }

  int tmp;
  if (!m_robothw_nh.getParam("maximum_missing_cycles", tmp))
  {
    addDiagnosticsMessage("WARN", "maximum_missing_cycles not defined, set equal to 50", {{"Transition", "switching"}}, &report);
    CNR_WARN(m_logger, report.str() );
    tmp = 50;
  }
  m_max_missing_messages = tmp;

  m_pos.resize(m_resource_names.size());
  m_vel.resize(m_resource_names.size());
  m_eff.resize(m_resource_names.size());

  std::fill(m_pos.begin(), m_pos.end(), 0.0);
  std::fill(m_vel.begin(), m_vel.end(), 0.0);
  std::fill(m_eff.begin(), m_eff.end(), 0.0);

  m_topic_received = false;
  m_first_topic_received = false;

  m_js_sub = m_robothw_nh.subscribe<sensor_msgs::JointState>(read_js_topic,
                                                             1,
                                                             &cnr_hardware_interface::TopicRobotHW::jointStateCallback,
                                                             this);

  m_js_pub = m_robothw_nh.advertise<sensor_msgs::JointState>(write_js_topic, 1);

  double timeout = 10;
  if (!m_robothw_nh.getParam("feedback_joint_state_timeout", timeout))
  {
    addDiagnosticsMessage("WARN", "feedback_joint_state_timeout not defined, set equal to 10", {{"Transition", "switching"}}, &report);
    CNR_WARN(m_logger, report.str() );

    timeout = 10;
  }

  m_cmd_pos.resize(m_resource_names.size());
  m_cmd_vel.resize(m_resource_names.size());
  m_cmd_eff.resize(m_resource_names.size());

  m_cmd_pos = m_pos;
  m_cmd_vel = m_vel;
  m_cmd_pos = m_eff;

  for (const std::string& joint_name : m_resource_names)
  {
    auto i = &joint_name - &m_resource_names[0];

    hardware_interface::JointStateHandle state_handle(joint_name, &(m_pos.at(i)), &(m_vel.at(i)), &(m_eff.at(i)));

    m_js_jh.registerHandle(state_handle);

    m_p_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_pos.at(i))));
    m_v_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_vel.at(i))));
    m_e_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_eff.at(i))));

    m_pve_jh.registerHandle(hardware_interface::PosVelEffJointHandle(state_handle, &(m_cmd_pos.at(i)), &(m_cmd_vel.at(i)), &(m_cmd_eff.at(i))));
    m_ve_jh.registerHandle(hardware_interface::VelEffJointHandle(state_handle, &(m_cmd_vel.at(i)), &(m_cmd_eff.at(i))));
  }

  registerInterface(&m_js_jh);
  registerInterface(&m_p_jh);
  registerInterface(&m_v_jh);
  registerInterface(&m_e_jh);
  registerInterface(&m_pve_jh);
  registerInterface(&m_ve_jh);

  m_p_jh_active = m_v_jh_active = m_e_jh_active = false;

  m_msg.reset(new sensor_msgs::JointState());
  m_msg->name = m_resource_names;
  m_msg->position = m_pos;
  m_msg->velocity = m_vel;
  m_msg->effort.resize(m_resource_names.size());
  std::fill(m_msg->effort.begin(), m_msg->effort.end(), 0.0);

  m_start_time = ros::Time::now();

  CNR_RETURN_TRUE(m_logger);
}

void TopicRobotHW::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(m_mutex);
  std::vector<std::string> names = msg->name;
  std::vector<double>      pos  = msg->position;
  std::vector<double>      vel  = msg->velocity;
  std::vector<double>      eff  = msg->effort;

  if((pos.size() < m_resource_names.size()) || (vel.size() < m_resource_names.size())
  || (eff.size() < m_resource_names.size()) || (names.size() < m_resource_names.size()))
  {
    std::string s = "Topic '" + m_js_sub.getTopic();
    s += " [Num publisher: " + std::to_string(m_js_pub.getNumSubscribers()) + "]' ";
    s += "Mismatch in msg size: p:" + std::to_string((int)(pos.size())) + ", v:"  + std::to_string((int)(vel.size()))
      + ", e:"  + std::to_string((int)(eff.size())) + ", names:" + std::to_string((int)(m_resource_names.size()));
    CNR_ERROR_THROTTLE(m_logger, 5.0, s);
    m_topic_received = false;
    return;
  }

  if (!name_sorting::permutationName(m_resource_names, names, pos, vel, eff, "ITIA TOPIC HW - jointStateCallback"))
  {
    m_topic_received = false;
    CNR_WARN_THROTTLE(m_logger, 0.1, m_robot_name << "Feedback joint states names are wrong!");
    return;
  }
  m_topic_received = true;

  for (unsigned int idx = 0; idx < m_resource_names.size(); idx++)
  {
    m_pos.at(idx) = pos.at(idx);
    m_vel.at(idx) = vel.at(idx);
    m_eff.at(idx) = eff.at(idx);
  }
  if(!m_first_topic_received)
  {
    m_first_topic_received = true;
    m_cmd_pos = m_pos;
    m_cmd_vel = m_vel;
    m_cmd_eff = m_eff;
  }
}

bool TopicRobotHW::doRead(const ros::Time& time, const ros::Duration& period)
{
  std::stringstream report;
  if ((!m_topic_received) && ((time - m_start_time).toSec() > 0.1))
  {
    m_missing_messages++;
  }
  else
  {
    m_missing_messages = 0;
  }

  m_topic_received = false;
  if(m_warmup < m_max_missing_messages*100)
  {

  }
  else if (m_missing_messages > m_max_missing_messages)
  {
    if (getStatus() == cnr_hardware_interface::RUNNING)
    {
      addDiagnosticsMessage("ERROR", "maximum_missing_cycles " + std::to_string(m_missing_messages) + "s ", {{"read", "missing messages"}}, &report);
      CNR_ERROR(m_logger, report.str() );
      return false;
    }
  }
  return true;
}

bool TopicRobotHW::doWrite(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(m_logger);
  if (!m_p_jh_active && !m_v_jh_active && !m_e_jh_active)
  {
    CNR_RETURN_TRUE_THROTTLE(m_logger, 5.0);
  }

  if(m_first_topic_received)
  {
    if(m_p_jh_active)
    {
      m_msg->position = m_cmd_pos;
    }

    if(m_v_jh_active)
    {
      m_msg->velocity = m_cmd_vel;
    }
    else
    {
      m_msg->velocity.resize(m_resource_names.size());
      std::fill(m_msg->velocity.begin(), m_msg->velocity.end(), 0.0);
    }

    if (m_e_jh_active)
    {
      m_msg->effort   = m_cmd_eff;
    }
    else
    {
      m_msg->effort.resize(m_resource_names.size());
      std::fill(m_msg->effort.begin(), m_msg->effort.end(), 0.0);
    }
    m_msg->name = m_resource_names;
    m_msg->header.stamp = ros::Time::now();

    m_js_pub.publish(m_msg);

    sensor_msgs::JointStatePtr msg(new sensor_msgs::JointState());
    m_msg.swap(msg);
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(m_logger);
}

bool TopicRobotHW::doPrepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
                                   const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  CNR_TRACE_START(m_logger);
  bool p_jh_active, v_jh_active, e_jh_active;

  p_jh_active = m_p_jh_active;
  v_jh_active = m_v_jh_active;
  e_jh_active = m_e_jh_active;

  for (const hardware_interface::ControllerInfo& controller : stop_list)
  {
    for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
    {
      if (!res.hardware_interface.compare("hardware_interface::PositionJointInterface"))
        p_jh_active = false;
      if (!res.hardware_interface.compare("hardware_interface::VelocityJointInterface"))
        v_jh_active = false;
      if (!res.hardware_interface.compare("hardware_interface::EffortJointInterface"))
        e_jh_active = false;
      if (!res.hardware_interface.compare("hardware_interface::VelEffJointInterface"))
      {
        v_jh_active = false;
        e_jh_active = false;
      }
      if (!res.hardware_interface.compare("hardware_interface::PosVelEffJointInterface"))
      {
        p_jh_active = false;
        v_jh_active = false;
        e_jh_active = false;
      }
    }
  }
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
  CNR_RETURN_TRUE(m_logger);
}

bool TopicRobotHW::doShutdown()
{
  if (!m_shutted_down)
  {
    m_js_sub.shutdown();
  }
  return true;
}

}  // namespace cnr_hardware_interface
