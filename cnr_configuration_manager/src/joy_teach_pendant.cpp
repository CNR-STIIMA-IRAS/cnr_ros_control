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
#include "ros/ros.h"
#include <configuration_msgs/ListConfigurations.h>
#include <configuration_msgs/StartConfiguration.h>
#include <configuration_msgs/StopConfiguration.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Joy.h>
#include <subscription_notifier/subscription_notifier.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "configuration_joy_interface");
  ros::NodeHandle nh;
  ros::Rate lp(100);
  ros::AsyncSpinner spin(4);
  spin.start();

  ros::ServiceClient list_config  = nh.serviceClient<configuration_msgs::ListConfigurations>("/configuration_manager/list_configurations");
  ros::ServiceClient start_config = nh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  ros::ServiceClient stop_config  = nh.serviceClient<configuration_msgs::StopConfiguration> ("/configuration_manager/stop_configuration");

  ros::ServiceClient srv_button0  = nh.serviceClient<std_srvs::SetBool>("/gripper/grasp");
  bool button0 = false;

  ros::ServiceClient pause  = nh.serviceClient<std_srvs::SetBool>("/pause");
  ros::Publisher ovr_pub = nh.advertise<std_msgs::Int64>("speed_ovr", 1);
  ros_helper::SubscriptionNotifier<sensor_msgs::Joy> joy_rec(nh, "joy", 1);

  ROS_INFO("Waiting for configuration_manager wake up...");
  list_config.waitForExistence();
  start_config.waitForExistence();
  stop_config.waitForExistence();

  int ovr_value = 10;
  std_msgs::Int64 ovr;
  for (int i = 0; i < 10; i++)
  {
    ovr_pub.publish(ovr);
    lp.sleep();
  }

  while ((ros::ok()))
  {
    if (joy_rec.isANewDataAvailable())
    {
      sensor_msgs::Joy joy = joy_rec.getData();
      if (joy.buttons.at(0))
      {
        std_srvs::SetBool srv;
        button0 = !button0;
        srv.request.data = button0;
        if (button0)
          ROS_INFO("grasp");
        else
          ROS_INFO("release");
        srv_button0.call(srv);
      }
      if (joy.buttons.at(4))
      {
        std_srvs::SetBool srv;

        srv.request.data = true;
        pause.call(srv);
      }
      if (joy.buttons.at(6))
      {
        std_srvs::SetBool srv;
        srv.request.data = false;
        pause.call(srv);
      }

      if (joy.buttons.at(5))
      {
        ovr.data += 5;
        ovr_value = std::min(100, std::max(0, ovr_value + 1));
      }
      if (joy.buttons.at(7))
      {
        ovr.data += 5;
        ovr_value = std::min(100, std::max(0, ovr_value - 1));
      }
      if (joy.buttons.at(0))
      {
        // switch configuration
      }
      if (joy.buttons.at(10))
      {
        configuration_msgs::StartConfiguration srv;
        srv.request.start_configuration = "watch";
        start_config.call(srv);
      }
      if (joy.buttons.at(11))
      {
        configuration_msgs::StartConfiguration srv;
        srv.request.start_configuration = "only_ur10";
        start_config.call(srv);
      }

    }
    ovr.data = ovr_value;
    ovr_pub.publish(ovr);
    lp.sleep();
  }
  return 0;
}
