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
#include <thread>


void ui_thread_fun()
{
  ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
  ros::NodeHandle nh;
  ros::AsyncSpinner spin(4);
  spin.start();

  ros::ServiceClient list_config  = nh.serviceClient<configuration_msgs::ListConfigurations>("/configuration_manager/list_configurations");
  ros::ServiceClient start_config = nh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  ros::ServiceClient stop_config  = nh.serviceClient<configuration_msgs::StopConfiguration>("/configuration_manager/stop_configuration");

  ROS_INFO("Waiting for configuration_manager wake up...");
  list_config.waitForExistence();
  start_config.waitForExistence();
  stop_config.waitForExistence();
  ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");

  while ((ros::ok()))
  {
    if (!list_config.isValid())
    {
      ROS_ERROR("Unable to connect with server '%s', probably the configuration_manager is died", list_config.getService().c_str());
      return;
    }
    configuration_msgs::ListConfigurations list;
    if (!list_config.call(list))
    {
      ROS_ERROR("Unable to obtain configuration list, probably the configuration_manager is died");
      return;
    }
    if (list.response.configurations.size() == 0)
    {
      ROS_ERROR("No configurations available");
      return;
    }

    ROS_INFO("Select the desired configuration:");
    int icfg = 0;
    ROS_INFO("-2) stop running configuration and EXIT");
    ROS_INFO("-1) stop running configuration");
    for (configuration_msgs::ConfigurationComponent& cfg : list.response.configurations)
    {
      ROS_INFO("%d) %s (%s)", icfg++, cfg.name.c_str(), cfg.state.c_str());
    }

    std::string input;
    std::cin >> input;

    int x;
    try
    {
      x = boost::lexical_cast<int>(input);
    }
    catch (boost::bad_lexical_cast const&)
    {
      std::cout << "Error: input string was not valid" << std::endl << std::endl << std::endl;
      continue;
    }
    if (x == -2)
    {
      ROS_INFO("stop active configuration\n\n\n");
      configuration_msgs::StopConfiguration srv;
      srv.request.strictness = 1;
      stop_config.call(srv);
      return;

    }
    else if (x == -1)
    {
      ROS_INFO("stop active configuration\n\n\n");
      configuration_msgs::StopConfiguration srv;
      srv.request.strictness = 1;
      stop_config.call(srv);

    }
    else if ((x < 0) || (x >= list.response.configurations.size()))
    {
      ROS_ERROR("out of bound\n\n\n");
      continue;
    }
    else
    {
      ROS_INFO("selected %d) %s\n\n\n", x, list.response.configurations.at(x).name.c_str());
      configuration_msgs::StartConfiguration srv;
      srv.request.start_configuration = list.response.configurations.at(x).name;
      srv.request.strictness = 1;
      start_config.call(srv);

    }

  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "configuration_user_interface");
  ros::NodeHandle nh;

  ros::ServiceClient list_config  =
      nh.serviceClient<configuration_msgs::ListConfigurations>("/configuration_manager/list_configurations");

  ROS_INFO("Waiting for configuration_manager wake up...");
  list_config.waitForExistence();

  ROS_INFO("Start thread...");
  std::thread ui_thread(ui_thread_fun);
  while (ros::ok() && list_config.exists() && ui_thread.joinable())
  {
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("shutting down..");
  if (ui_thread.joinable())
  {
    ROS_INFO("kill thread");
    std::terminate();
    ui_thread.join();
    ROS_INFO("done");
  }


  return 0;
}
