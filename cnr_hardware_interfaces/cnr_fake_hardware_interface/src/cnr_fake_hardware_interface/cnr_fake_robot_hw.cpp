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
#include <pluginlib/class_list_macros.h>


#include <cnr_controller_interface_params/cnr_controller_interface_params.h>
#include <cnr_hardware_interface/internal/vector_to_string.h>
#include <cnr_hardware_interface/cnr_robot_hw.h>
#include <cnr_fake_hardware_interface/cnr_fake_robot_hw.h>


PLUGINLIB_EXPORT_CLASS(cnr_hardware_interface::FakeRobotHW, cnr_hardware_interface::RobotHW)

namespace cnr_hardware_interface
{

void setParam(FakeRobotHW* hw, const std::string& ns)
{
  hw->m_robothw_nh.setParam("status/" + ns + "/feedback/name", hw->resourceNames());
  hw->m_robothw_nh.setParam("status/" + ns + "/feedback/position", hw->m_pos);
  hw->m_robothw_nh.setParam("status/" + ns + "/feedback/velocity", hw->m_vel);
  hw->m_robothw_nh.setParam("status/" + ns + "/feedback/effort", hw->m_eff);
  hw->m_robothw_nh.setParam("status/" + ns + "/command/name", hw->resourceNames());
  hw->m_robothw_nh.setParam("status/" + ns + "/command/position", hw->m_cmd_pos);
  hw->m_robothw_nh.setParam("status/" + ns + "/command/velocity", hw->m_cmd_vel);
  hw->m_robothw_nh.setParam("status/" + ns + "/command/effort", hw->m_cmd_eff);
}

FakeRobotHW::FakeRobotHW()
  : m_msg(nullptr)
{
  m_set_status_param = boost::bind(setParam, this, _1);
}

FakeRobotHW::~FakeRobotHW()
{
  if (!m_shutted_down)
  {
    shutdown();
  }
}

void FakeRobotHW::initialJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  m_msg  = new sensor_msgs::JointState();
  *m_msg = *msg;
}

void FakeRobotHW::wrenchCb(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  if (msg->header.frame_id.compare(m_frame_id))
  {
    CNR_WARN_THROTTLE(m_logger,1,"wrench has wrong frame_id. expected = "<<m_frame_id<<", received = "<<msg->header.frame_id);
    return;
  }
  m_ft_sensor.at(0)=msg->wrench.force.x;
  m_ft_sensor.at(1)=msg->wrench.force.y;
  m_ft_sensor.at(2)=msg->wrench.force.z;
  m_ft_sensor.at(3)=msg->wrench.torque.x;
  m_ft_sensor.at(4)=msg->wrench.torque.y;
  m_ft_sensor.at(5)=msg->wrench.torque.z;

  CNR_TRACE_THROTTLE(m_logger,10,"received a wrench");
}

bool FakeRobotHW::doInit()
{
  CNR_TRACE_START(m_logger);

  CNR_DEBUG(m_logger, "Resources (" << resourceNumber() << "): " << cnr_hardware_interface::to_string(resourceNames()));
  m_pos.resize(resourceNumber());
  m_vel.resize(resourceNumber());
  m_eff.resize(resourceNumber());

  std::fill(m_pos.begin(), m_pos.end(), 0.0);
  std::fill(m_vel.begin(), m_vel.end(), 0.0);
  std::fill(m_eff.begin(), m_eff.end(), 0.0);

  m_ft_sensor.resize(6);
  std::fill(m_ft_sensor.begin(), m_ft_sensor.end(), 0.0);

  if (m_robothw_nh.hasParam("initial_position"))
  {
    m_robothw_nh.getParam("initial_position", m_pos);
    std::string ss;
    for (auto const & p : m_pos) ss += std::to_string(p) + ", ";
    CNR_DEBUG(m_logger, "Initial Position: <" << ss << ">");
  }
  else if (m_robothw_nh.hasParam("initial_position_from"))
  {
    std::string position_from;
    m_robothw_nh.getParam("position_from", position_from);

    CNR_DEBUG(m_logger, "Position From: '" << position_from << "'");
    std::string position_ns = "/" + position_from + "/status/shutdown_configuration/position";
    if (!m_robothw_nh.hasParam(position_ns))
    {
      CNR_ERROR(m_logger, "The param '" + position_ns + "' does not exit. pos superimposed to zero");
    }
    m_robothw_nh.getParam(position_ns, m_pos);
    std::string ss;
    for (auto const & p : m_pos) ss += std::to_string(p) + ", ";
    CNR_DEBUG(m_logger, "Initial Position: <" << ss << ">");
  }

  double timeout = 10;
  if (!m_robothw_nh.getParam("feedback_joint_state_timeout", timeout))
  {
    CNR_WARN(m_logger, "The param '" << m_robothw_nh.getNamespace() << "/feedback_joint_state_timeout' not defined, set equal to 10");
    timeout = 10;
  }

  m_cmd_pos.resize(resourceNumber());
  m_cmd_vel.resize(resourceNumber());
  m_cmd_eff.resize(resourceNumber());

  m_cmd_pos = m_pos;
  m_cmd_vel = m_vel;
  m_cmd_pos = m_eff;

  for(size_t i=0;i<resourceNumber();i++)
  {
    std::string joint_name = resourceNames().at(i);
    //auto i = &joint_name - &m_resource_names[0];

    hardware_interface::JointStateHandle state_handle(joint_name, &(m_pos.at(i)), &(m_vel.at(i)), &(m_eff.at(i)));

    m_js_jh.registerHandle(state_handle);

    m_p_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_pos.at(i))));
    m_v_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_vel.at(i))));
    m_e_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_eff.at(i))));

    m_pve_jh.registerHandle(hardware_interface::PosVelEffJointHandle(state_handle, &(m_cmd_pos.at(i)), &(m_cmd_vel.at(i)), &(m_cmd_eff.at(i))));
    m_ve_jh.registerHandle(hardware_interface::VelEffJointHandle(state_handle, &(m_cmd_vel.at(i)), &(m_cmd_eff.at(i))));
  }

  std::string wrench_name="wrench";
  if (!m_robothw_nh.getParam("wrench_resourse",wrench_name))
  {
    wrench_name="wrench";
    CNR_TRACE(m_logger,"using defalut wrench_resourse name: wrench");
  }

  if (!m_robothw_nh.getParam("frame_id",m_frame_id))
  {
    m_frame_id="tool0";
    CNR_TRACE(m_logger,"using defalut frame_id name: tool0");
  }

  std::string wrench_topic="fake_wrench";
  if (!m_robothw_nh.getParam("wrench_topic",wrench_topic))
  {
    wrench_topic="fake_wrench";
    CNR_TRACE(m_logger,"using defalut wrench_topic name: fake_wrench");
  }
  m_wrench_sub=m_robothw_nh.subscribe(wrench_topic,1,&FakeRobotHW::wrenchCb,this);


  hardware_interface::ForceTorqueSensorHandle sensor_handle(wrench_name
      , m_frame_id
      , &(m_ft_sensor.at(0))
      , &(m_ft_sensor.at(3)));

  m_ft_jh.registerHandle(sensor_handle);

  registerInterface(&m_js_jh);
  registerInterface(&m_p_jh);
  registerInterface(&m_v_jh);
  registerInterface(&m_e_jh);
  registerInterface(&m_pve_jh);
  registerInterface(&m_ve_jh);
  registerInterface(&m_ft_jh);

  m_p_jh_active = m_v_jh_active = m_e_jh_active = false;

  doRead(ros::Time::now(),ros::Duration(m_sampling_period));

  CNR_RETURN_TRUE(m_logger);
}

bool FakeRobotHW::doWrite(const ros::Time& /*time*/, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(m_logger);
  if(m_p_jh_active)
  {
    m_pos = m_cmd_pos;
  }

  if (m_v_jh_active)
  {
    m_vel = m_cmd_vel;
    if(!m_p_jh_active)
    {
      for(size_t iAx=0; iAx<resourceNumber(); iAx++)
      {
        m_pos.at(iAx) = m_pos.at(iAx) + m_cmd_vel.at(iAx) * period.toSec();
      }
    }
  }
  else
  {
    m_vel.resize(resourceNumber());
    std::fill(m_vel.begin(), m_vel.end(), 0.0);
  }

  if (m_e_jh_active)
  {
    m_eff = m_cmd_eff;
  }
  else
  {
    m_eff.resize(resourceNumber());
    std::fill(m_eff.begin(), m_eff.end(), 0.0);
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(m_logger);
}

bool FakeRobotHW::doPrepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
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

  std::vector<std::string> resources;
  for (const hardware_interface::ControllerInfo& controller : start_list)
  {
    for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
    {
      resources.push_back(res.hardware_interface);
      CNR_DEBUG(m_logger, "Claimed resource: " << res.hardware_interface);
      p_jh_active = (res.hardware_interface == "hardware_interface::PositionJointInterface")
                    || (res.hardware_interface == "hardware_interface::PosVelEffJointInterface")
                    ? true : p_jh_active;

      v_jh_active = (res.hardware_interface == "hardware_interface::VelocityJointInterface")
                    || (res.hardware_interface == "hardware_interface::VelEffJointInterface")
                    || (res.hardware_interface == "hardware_interface::PosVelEffJointInterface")
                    ? true : v_jh_active;

      e_jh_active = (res.hardware_interface == "hardware_interface::EffortJointInterface")
                    || (res.hardware_interface == "hardware_interface::VelEffJointInterface")
                    || (res.hardware_interface == "hardware_interface::PosVelEffJointInterface")
                    ? true : e_jh_active;
    }
  }
  m_p_jh_active = p_jh_active;
  m_v_jh_active = v_jh_active;
  m_e_jh_active = e_jh_active;
  CNR_DEBUG(m_logger, " Pos joint handle active? " << m_p_jh_active);
  CNR_DEBUG(m_logger, " Vel joint handle active? " << m_v_jh_active);
  CNR_DEBUG(m_logger, " Eff joint handle active? " << m_e_jh_active);
  CNR_RETURN_TRUE(m_logger, "Active hardware interfaces: " + cnr::control::to_string(resources));

}

bool FakeRobotHW::doCheckForConflict(const std::list< hardware_interface::ControllerInfo >& info)
{
  std::stringstream report;
  CNR_TRACE_START(m_logger);
  // Each controller can use more than one hardware_interface for a single joint (e.g.: position, velocity, effort).
  // One controller can control more than one joint.
  // A joint can be used only by a controller.

  std::vector<bool> global_joint_used(resourceNumber());
  std::fill(global_joint_used.begin(), global_joint_used.end(), false);

  for (hardware_interface::ControllerInfo controller : info)
  {
    std::vector<bool> single_controller_joint_used(resourceNumber());
    std::fill(single_controller_joint_used.begin(), single_controller_joint_used.end(), false);

    for (hardware_interface::InterfaceResources res : controller.claimed_resources)
    {
      for (std::string name : res.resources)
      {
        for (unsigned int iJ = 0; iJ < resourceNumber(); iJ++)
        {
          if (!name.compare(resourceNames().at(iJ)))
          {
            if (global_joint_used.at(iJ)) // if already used by another
            {
              addDiagnosticsMessage("ERROR", "Joint " + name + " is already used by another controller", {{"Transition", "switching"}}, &report);
              CNR_ERROR(m_logger, report.str());
              CNR_RETURN_TRUE(m_logger, "Joint " + name + " is already used by another controller");
            }
            else
            {
              single_controller_joint_used.at(iJ) = true;
            }
          }
        }
      }
    }
    for (unsigned int iJ = 0; iJ < resourceNumber(); iJ++)
    {
      global_joint_used.at(iJ) = global_joint_used.at(iJ) || single_controller_joint_used.at(iJ);
    }

  }
  CNR_RETURN_FALSE(m_logger);
}



}
