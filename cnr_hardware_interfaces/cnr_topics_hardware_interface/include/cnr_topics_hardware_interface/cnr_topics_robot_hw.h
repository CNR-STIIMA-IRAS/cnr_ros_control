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
#ifndef CNR_HARDWARE_INTERFACE_CNR_TOPICS_ROBOT_HW_H
#define CNR_HARDWARE_INTERFACE_CNR_TOPICS_ROBOT_HW_H


#include <mutex>
#include <ros/time.h>
#include <hardware_interface/controller_info.h>
#include <cnr_hardware_interface/cnr_robot_hw.h>
#include <cnr_topics_hardware_interface/claimed_resources.h>

namespace cnr_hardware_interface
{

class TopicsRobotHW: public cnr_hardware_interface::RobotHW
{
public:
  TopicsRobotHW();
  virtual ~TopicsRobotHW()
  {
    if (!m_shutted_down)
      shutdown();
  }
  virtual bool doShutdown() override;
  virtual bool doRead(const ros::Time& time, const ros::Duration& period) override;
  virtual bool doWrite(const ros::Time& time, const ros::Duration& period) override;

  virtual bool doInit() override;
  virtual bool doCheckForConflict(const std::list<hardware_interface::ControllerInfo>& info) const override;
  virtual bool doPrepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, 
                               const std::list<hardware_interface::ControllerInfo>& stop_list) override;


protected:

  std::map< RESOURCE_ID, std::shared_ptr< cnr_hardware_interface::Resource > > m_resources;

  std::shared_ptr< JointClaimedResource > m_joint_resource;
  bool initJointClaimedResource();

  std::shared_ptr< AnalogClaimedResource > m_analog_resource;
  bool initAnalogClaimedResource();

  std::shared_ptr< ForceTorqueClaimedResource > m_force_torque_sensor_resource;
  bool initForceTorqueClaimedResource();

  std::shared_ptr< PoseClaimedResource > m_pose_resource;
  bool initPoseClaimedResource();

  std::shared_ptr< TwistClaimedResource > m_twist_resource;
  bool initTwistClaimedResource();


  enum status { created, initialized, run, error };

  unsigned int                  m_warmup;
  unsigned int                  m_missing_messages;
  unsigned int                  m_max_missing_messages;
  std::map< std::string, bool>  m_topics_subscribed;

  bool allSubscriberConnected() const
  {
    bool all_topics_received = true;
    for (auto const & topic_received : m_topics_subscribed) all_topics_received &= topic_received.second;
    return all_topics_received;
  }

  bool topicsReceived() const
  {
    bool all_topics_received = true;
    for (auto const & topic_received : m_topics_subscribed) all_topics_received &= topic_received.second;
    return all_topics_received;
  }
  void resetTopicsReceived()
  {
    for (auto & topic_received : m_topics_subscribed) topic_received.second = false;
  }

  friend void setParam(TopicsRobotHW* hw, const std::string& ns);

};

void setParam(TopicsRobotHW* hw, const std::string& ns);
std::vector<std::string> getResourceNames(const std::map< cnr_hardware_interface::RESOURCE_ID, std::shared_ptr< cnr_hardware_interface::Resource > >& resources) ;

}  // namespace cnr_hardware_interface

#endif  // CNR_HARDWARE_INTERFACE_CNR_TOPICS_ROBOT_HW_H
