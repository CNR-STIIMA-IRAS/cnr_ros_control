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
#ifndef CNR_HARDWARE_INTERFACE_CNR_FAKE_ROBOT_HW
#define CNR_HARDWARE_INTERFACE_CNR_FAKE_ROBOT_HW

#include <mutex>

#include <cnr_hardware_interface/cnr_robot_hw.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace cnr_hardware_interface
{

class FakeRobotHW: public cnr_hardware_interface::RobotHW
{
public:
  FakeRobotHW();
  ~FakeRobotHW();

  virtual bool doInit();
  virtual bool doWrite(const ros::Time& time, const ros::Duration& period);

  virtual bool doPrepareSwitch(const std::list< hardware_interface::ControllerInfo >& start, 
                               const std::list< hardware_interface::ControllerInfo >& stop);
  virtual bool doCheckForConflict(const std::list<hardware_interface::ControllerInfo>& info);

protected:
  sensor_msgs::JointState* m_msg;
  void initialJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  hardware_interface::JointStateInterface    m_js_jh;   //interface for reading joint state
  hardware_interface::PositionJointInterface m_p_jh;    //interface for writing position target
  hardware_interface::VelocityJointInterface m_v_jh;    //interface for writing velocity target
  hardware_interface::EffortJointInterface   m_e_jh;    //interface for writing effort target
  hardware_interface::PosVelEffJointInterface m_pve_jh;
  hardware_interface::VelEffJointInterface m_ve_jh;

  bool m_p_jh_active;
  bool m_v_jh_active;
  bool m_e_jh_active;

  std::vector<double> m_pos; // feedback position
  std::vector<double> m_vel; // feedback velocity
  std::vector<double> m_eff; // feedback effort

  std::vector<double> m_cmd_pos; //target position
  std::vector<double> m_cmd_vel; //target velocity
  std::vector<double> m_cmd_eff; //target effort

  std::mutex m_mutex;

  enum status { created, initialized, run, error };

  friend void setParam(FakeRobotHW* hw, const std::string& ns);

};

void setParam(FakeRobotHW* hw, const std::string& ns);

}  // namespace cnr_hardware_interface

#endif  //  CNR_HARDWARE_INTERFACE_CNR_FAKE_ROBOT_HW
