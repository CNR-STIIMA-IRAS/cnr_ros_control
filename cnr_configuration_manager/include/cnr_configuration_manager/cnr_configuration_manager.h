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
#ifndef CNR_CONFIGURATION_MANAGER_CNR_CONFIGURATION_MANAGER_H
#define CNR_CONFIGURATION_MANAGER_CNR_CONFIGURATION_MANAGER_H

#include <nodelet/nodelet.h>
#include <nodelet/loader.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletList.h>

#include <configuration_msgs/StartConfiguration.h>
#include <configuration_msgs/StopConfiguration.h>
#include <configuration_msgs/ListConfigurations.h>
#include <configuration_msgs/UpdateConfigurations.h>

#include <cnr_logger/cnr_logger.h>
#include <cnr_hardware_interface/internal/cnr_robot_hw_utils.h>
#include <cnr_controller_manager_interface/cnr_controller_manager_interface.h>
#include <cnr_configuration_manager/internal/cnr_configuration_manager_utils.h>
#include <cnr_configuration_manager/cnr_configuration_loader.h>

namespace cnr_configuration_manager
{


class ConfigurationManager
{
public:

  ConfigurationManager(std::shared_ptr<cnr_logger::TraceLogger>& logger,
                       const ros::NodeHandle& nh) noexcept(false);

  ~ConfigurationManager() noexcept(false);

  bool startCallback(configuration_msgs::StartConfiguration::Request& req,
                     configuration_msgs::StartConfiguration::Response& res);

  bool stopCallback(configuration_msgs::StopConfiguration::Request& req,
                    configuration_msgs::StopConfiguration::Response& res);

  bool listConfigurations(configuration_msgs::ListConfigurations::Request& req,
                          configuration_msgs::ListConfigurations::Response& res);

  bool updateConfigurations(configuration_msgs::UpdateConfigurations::Request& req,
                            configuration_msgs::UpdateConfigurations::Response& res);

  bool init();

  bool run();

  bool isOk(bool nodelet_check);

private:
  ros::NodeHandle                                  m_nh;
  std::shared_ptr<cnr_logger::TraceLogger>         m_logger;
  std::mutex                                       m_callback_mutex;
  std::string                                      m_active_configuration_name;
  ConfigurationStruct                              m_active_configuration;
  std::map<std::string, ConfigurationStruct>       m_configurations;

  ConfigurationLoader                              m_conf_loader;

  ros::ServiceServer                               m_load_configuration      ;
  ros::ServiceServer                               m_unload_configuration    ;
  ros::ServiceServer                               m_list_controller_service ;

  SignalHandler                                    m_signal_handler;

  bool checkRobotHwState(const std::string& hw, cnr_hardware_interface::StatusHw target = cnr_hardware_interface::RUNNING);
  bool callback(const ConfigurationStruct& next_configuration, const int &strictness, const ros::Duration& watchdog);
  bool getAvailableConfigurationsFromParam();
};

}  // namespace cnr_configuration_manager


#endif  // CNR_CONFIGURATION_MANAGER_CNR_CONFIGURATION_MANAGER_H
