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
#include <std_srvs/Empty.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Joy.h>
#include <subscription_notifier/subscription_notifier.h>
#include <cnr_logger/cnr_logger.h>


void gazebo_cb(const ros::TimerEvent& ev)
{
  ros::NodeHandle nh;
  std_srvs::Empty srv;
  ros::ServiceClient start_gazebo_srv  = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  start_gazebo_srv.call(srv);
  ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><Gazebo unpaused");
}

class Dispatch
{
protected:
  ros::ServiceClient m_start_config;
  configuration_msgs::StartConfiguration conf_srv;
  cnr_logger::TraceLogger& m_logger;
public:
  Dispatch(const std::string& configuration, const ros::ServiceClient& start_config, cnr_logger::TraceLogger& logger):
    m_start_config(start_config), m_logger(logger)
  {
    conf_srv.request.start_configuration = configuration;
    conf_srv.request.strictness = 1;
    CNR_INFO(m_logger, "Constructor: default configuration " << conf_srv.request.start_configuration);

  }
  void dispatch(const ros::TimerEvent& e)
  {
    CNR_INFO(m_logger, "Timer Callback: starting configuration " << conf_srv.request.start_configuration);
    m_start_config.call(conf_srv);
    CNR_INFO(m_logger, "Timer Callback: starting configuration "
             << conf_srv.request.start_configuration << " " << (conf_srv.response.ok ? "DONE" : "FAILED"));
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "configuration_dispatcher");
  ros::start();

  ros::NodeHandle nh("~");
  ros::ServiceClient list_config  = nh.serviceClient<configuration_msgs::ListConfigurations>("/configuration_manager/list_configurations");
  ros::ServiceClient start_config = nh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  ros::ServiceClient stop_config  = nh.serviceClient<configuration_msgs::StopConfiguration> ("/configuration_manager/stop_configuration");

  std::string n = ros::this_node::getName();
  n.erase(0, 1);
  std::replace(n.begin(), n.end(), '/', '_');


  cnr_logger::TraceLogger logger(n, nh.getNamespace());

  bool start_gazebo;
  if (!nh.getParam("start_gazebo", start_gazebo))
    start_gazebo = false;

  ros::Timer gazebo_timer;
  if (start_gazebo)
  {
    double gazebo_wait_time = 4;
    if (!nh.getParam("gazebo_bringup_time", gazebo_wait_time))
      gazebo_wait_time = 4;
    //gazebo_timer = nh.createWallTimer(ros::Duration(gazebo_wait_time), gazebo_cb, true);

    CNR_INFO(logger, "Waiting " << gazebo_wait_time << "seconds before unpausing gazebo (timer started: " << gazebo_timer.hasStarted() << ") ");

  }
  else
  {
    CNR_INFO(logger, "Waiting for configuration_manager wake up...");
    list_config.waitForExistence();
    start_config.waitForExistence();
    stop_config.waitForExistence();

  }

  XmlRpc::XmlRpcValue param_configurations;

  if (!nh.getParam("configuration_dispatches", param_configurations))
  {
    CNR_WARN(logger, "Nothing to dispatch");
    return 0;
  }
  std::vector< std::pair<std::string, double> > configurations;
  double time_from_start = 0.0;
  for( int i=0; i< param_configurations.size(); i++)
  {
    XmlRpc::XmlRpcValue param_configuration = param_configurations[i];
    std::pair<std::string, double>  configuration;
    configuration.first = (std::string)(param_configuration["name"]);
    time_from_start += (double)(param_configuration["at"]);
    configuration.second = time_from_start;
    configurations.push_back(configuration);
  }


  std::vector<std::shared_ptr<Dispatch>> dispatches;
  std::vector<ros::Timer> dispatch_timers;
  for (const std::pair<std::string, double>&p : configurations)
  {
    dispatches.push_back(std::make_shared<Dispatch>(p.first, start_config, logger));
    dispatch_timers.push_back(nh.createTimer(ros::Duration(p.second),
                              &Dispatch::dispatch,
                              (dispatches.back()).get(),
                              true)) ;
  }
  ros::spin();

  return 0;
}
