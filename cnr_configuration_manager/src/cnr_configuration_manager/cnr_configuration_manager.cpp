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
#include <atomic>
#include <mutex>

#include <ros/ros.h>

#include <nodelet/loader.h>
#include <nodelet/loader.h>
#include <nodelet/NodeletList.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>

#include <cnr_logger/cnr_logger.h>

#include <controller_manager_msgs/ControllerState.h>

#include <configuration_msgs/ConfigurationComponent.h>
#include <configuration_msgs/StartConfiguration.h>
#include <configuration_msgs/StopConfiguration.h>
#include <configuration_msgs/ListConfigurations.h>
#include <configuration_msgs/UpdateConfigurations.h>

#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_configuration_manager/signal_handler.h>
#include <cnr_configuration_manager/internal/cnr_configuration_manager_utils.h>
#include <cnr_configuration_manager/internal/cnr_configuration_manager_xmlrpc.h>
#include <cnr_configuration_manager/cnr_configuration_loader.h>
#include <cnr_configuration_manager/cnr_configuration_manager.h>

namespace cnr_configuration_manager
{


ConfigurationManager::ConfigurationManager(std::shared_ptr<cnr_logger::TraceLogger>& logger,
                                           const ros::NodeHandle& nh) noexcept(false)
: m_nh(nh)
, m_active_configuration_name("None")
, m_logger(logger)
, m_conf_loader(m_logger, m_nh)
{

}

ConfigurationManager::~ConfigurationManager() noexcept(false)
{
  try
  {
    CNR_TRACE_START(*m_logger);

    std::string error;
    std::vector<std::string>  hw_names_from_nodelet;
    if (!m_conf_loader.listHw(hw_names_from_nodelet, ros::Duration(10)))
    {
      CNR_EXIT_EX(*m_logger, false, "Error in getting the loaded hardware interfaces by the nodelet manager: " + m_conf_loader.error());
    }

    CNR_DEBUG(*m_logger, "Unload all hw nodelet (" << to_string(hw_names_from_nodelet) << ")");
    if (!m_conf_loader.unloadHw(hw_names_from_nodelet, ros::Duration(10)))
    {
      CNR_FATAL(*m_logger, "Unload the configuration failed. Error: " + m_conf_loader.error());
    }

    CNR_INFO(*m_logger, "[ DONE]");
    m_logger.reset();
  }
  catch (std::exception& e)
  {
    std::cerr << "Error in destructor of the ConfigurationManager: " << e.what() << std::endl;
  }
  catch (...)
  {
    std::cerr << "Error in destructor of the ConfigurationManager: Unhandled exception" << std::endl;
  }
}

bool ConfigurationManager::startCallback(configuration_msgs::StartConfiguration::Request&   req, configuration_msgs::StartConfiguration::Response&   res)
{
  CNR_TRACE_START(*m_logger);
  CNR_INFO(*m_logger, "************************** ******************************* *******************************");
  CNR_INFO(*m_logger, "************************** Start Configuration: '" << req.start_configuration << "'");
  CNR_INFO(*m_logger, "************************** ******************************* *******************************");

  try
  {
    const std::lock_guard<std::mutex> lock(m_callback_mutex);
    if (m_configurations.find(req.start_configuration) == m_configurations.end())
    {
      res.ok = false;
      CNR_ERROR(*m_logger, "The Configuration '" + req.start_configuration + "' is not among the listed.");
    }
    else
    {
      ConfigurationStruct next_configuration  = m_configurations.at(req.start_configuration);
      res.ok = callback(&next_configuration, req.strictness,  ros::Duration(10.0));
      if (res.ok)
      {
        m_active_configuration      = next_configuration;
        m_active_configuration_name = req.start_configuration;
        m_nh.setParam("status/active_configuration", m_active_configuration_name);
      }
    }
  }
  catch (std::exception& e)
  {
    CNR_RETURN_FALSE(*m_logger, "Exception in start configuration: " + std::string(e.what()));
  }
  catch (...)
  {
    CNR_RETURN_FALSE(*m_logger, "Exception in start configuration. Service Failure.");
  }
  CNR_RETURN_TRUE(*m_logger);
}

bool ConfigurationManager::stopCallback(configuration_msgs::StopConfiguration::Request&    req, configuration_msgs::StopConfiguration::Response&    res)
{
  CNR_TRACE_START(*m_logger);
  CNR_INFO(*m_logger, "*************************** ******************************** ********************************");
  CNR_INFO(*m_logger, "*************************** Stop Configuration");
  CNR_INFO(*m_logger, "*************************** ******************************** ********************************");

  try
  {
    const std::lock_guard<std::mutex> lock(m_callback_mutex);
    res.ok = callback(nullptr, req.strictness, ros::Duration(10.0));
    if (res.ok)
    {
      m_active_configuration      = ConfigurationStruct();
      m_active_configuration_name = "";
      m_nh.setParam("status/active_configuration", m_active_configuration_name);
    }
  }
  catch (std::exception& e)
  {
    CNR_RETURN_FALSE(*m_logger, "Exception in stop configuration: " + std::string(e.what()));
  }
  catch (...)
  {
    CNR_RETURN_FALSE(*m_logger, "Exception in stop configuration. Service Failure.");
  }
  CNR_RETURN_TRUE(*m_logger);
}

bool ConfigurationManager::listConfigurations(configuration_msgs::ListConfigurations::Request&   req, configuration_msgs::ListConfigurations::Response&   res)
{
  CNR_TRACE_START(*m_logger);
  res.configurations.clear();
  const std::lock_guard<std::mutex> lock(m_callback_mutex);
  if (!updateConfigurations())
  {
    CNR_RETURN_FALSE(*m_logger, "Update COnfiguration Failed.");
  }

  res.configurations.resize(m_configurations.size());
  for (auto it = m_configurations.begin(); it != m_configurations.end(); it++)
  {
    size_t i = std::distance(m_configurations.begin(), it);
    if (!cast(it->second, res.configurations.at(i)))
    {
      CNR_RETURN_BOOL(*m_logger, false, "Cast Error");
    }
  }
  CNR_RETURN_TRUE(*m_logger);
}

bool ConfigurationManager::updateConfigurations(configuration_msgs::UpdateConfigurations::Request& req, configuration_msgs::UpdateConfigurations::Response& res)
{
  CNR_TRACE_START(*m_logger);
  const std::lock_guard<std::mutex> lock(m_callback_mutex);
  bool ret = updateConfigurations();
  CNR_RETURN_BOOL(*m_logger, ret);
}

bool ConfigurationManager::init()
{
  CNR_TRACE_START(*m_logger);
  try
  {

    CNR_WARN(*m_logger, "********************* INIT ****************************");
    m_load_configuration      = m_nh.advertiseService("start_configuration", &cnr_configuration_manager::ConfigurationManager::startCallback,      this);
    m_unload_configuration    = m_nh.advertiseService("stop_configuration", &cnr_configuration_manager::ConfigurationManager::stopCallback,       this);
    m_list_controller_service = m_nh.advertiseService("list_configurations", &cnr_configuration_manager::ConfigurationManager::listConfigurations, this);

    CNR_TRACE_START(*m_logger);
    if (m_nh.hasParam("control_configurations") && updateConfigurations())
    {
      m_nh.setParam("status/active_configuration", "none");
    }
    else
    {
      CNR_RETURN_FALSE(*m_logger, "Configuration Manager: param '" + m_nh.getNamespace() + "/control_configurations' is broken. Abort");
    }

    m_signal_handler.setupSignalHandlers();

  }
  catch (SignalException& e)
  {
    CNR_RETURN_FALSE(*m_logger, "Error in the initialization. Abort.");
  }
  CNR_RETURN_TRUE(*m_logger);
}

bool ConfigurationManager::run()
{
  CNR_TRACE_START(*m_logger);
  try
  {
    const int decimator = 100;
    ros::Rate lp(decimator);
    size_t cnt = 0;
    CNR_WARN(*m_logger, "********************* RUN ****************************");
    while (ros::ok())
    {
      bool full_check = (((cnt++) % decimator) == 0);
      if (!isOk(full_check))
      {
        CNR_WARN_THROTTLE(*m_logger, 2, "Raised an Error by one of the Hw. Try to Stop configuration because a error has been raised by one of the Hw ");
        configuration_msgs::StopConfiguration srv;
        srv.request.strictness = 1;
        if (!stopCallback(srv.request, srv.response))
        {
          CNR_WARN_THROTTLE(*m_logger, 2, "Impossible to Stop the Running Configuration. Broken Service.");
          return false;
        }
        if (!srv.response.ok)
        {
          CNR_WARN_THROTTLE(*m_logger, 2, "Impossible to Stop the Running Configuration. The service response is negative.");
          return false;
        }
      }
      else
      {
        CNR_WARN_THROTTLE(*m_logger, 10.0, "Waiting for a new callback, or some change in the cofiguration state");
      }

      if (m_signal_handler.gotExitSignal())
      {
        CNR_WARN_THROTTLE(*m_logger, 2, "SIGINT called. Controlled exit.");
        configuration_msgs::StopConfiguration stop;
        stop.request.strictness = 1;
        if (!stopCallback(stop.request, stop.response))
        {
          CNR_FATAL(*m_logger, "Error in stopping the configuration ....");
        }
        break;    // exit normally after SIGINT
      }
      lp.sleep();
    }
  }
  catch (std::exception& e)
  {
    std::cerr << "Configuration Manager - Exception: " << e.what() << std::endl;
  }
  CNR_RETURN_TRUE(*m_logger);
}

bool ConfigurationManager::isOk(bool nodelet_check)
{
  std::string error;
  try
  {
    for (auto const & component : m_active_configuration.components)
    {
      const std::string& hw = component.first;
      cnr_hardware_interface::StatusHw hw_status;

      if (!cnr_hardware_interface::get_state(m_nh, hw, hw_status, ros::Duration(0.01), error))
      {
        CNR_FATAL(*m_logger, "The HW " << hw << " has not any valid state: " << error);
        return false;
      }

      if ((hw_status == cnr_hardware_interface::ERROR)
       || (hw_status == cnr_hardware_interface::CTRL_ERROR)
       || (hw_status == cnr_hardware_interface::SRV_ERROR))
      {
        CNR_FATAL(*m_logger, "The status of the HW '" << hw << "' is " << cnr_hardware_interface::to_string(hw_status));
        return false;
      }
      for (auto const & ctrl : component.second)
      {
        std::string ctrl_status;
        if (!cnr_controller_interface::get_state(hw, ctrl, ctrl_status, error, ros::Duration(0.01)))
        {
          CNR_FATAL(*m_logger, "The HW '" << hw << "' and CTRL '" << ctrl << "' has not any valid state: " << error);
          return false;
        }
        if (ctrl_status == "ERROR")
        {
          CNR_FATAL(*m_logger, "The status of HW '" << hw << "' and CTRL '" << ctrl << "' is " << ctrl_status
                               << " while it should be 'RUNNING'");
          return false;
        }
      }
    }

    if (nodelet_check)
    {
      std::vector<std::string> hw_names_from_nodelet;
      if (!m_conf_loader.listHw(hw_names_from_nodelet, ros::Duration(0.1)))
      {
        CNR_FATAL(*m_logger, "HW Nodelet Manager failed: " << m_conf_loader.error());
        return false;
      }

      for (auto const & component : m_active_configuration.components)
      {
        const std::string& hw = component.first;
        if (std::find(hw_names_from_nodelet.begin(), hw_names_from_nodelet.end(), hw) == hw_names_from_nodelet.end())
        {
          CNR_FATAL(*m_logger, "HW " << hw << " seems not loaded in memory!");
          return false;
        }
        std::vector< controller_manager_msgs::ControllerState >  running;
        std::vector< controller_manager_msgs::ControllerState >  stopped;
        if (!m_conf_loader.listControllers(hw, running, stopped))
        {
          CNR_FATAL(*m_logger, "Ctrl of the HW" << hw << " seems not working properly. "
                               << "Error: " << m_conf_loader.error(hw));
          return false;
        }
        for (auto const & ctrl : component.second)
        {
          if (std::find_if(running.begin(), running.end(), [&ctrl](auto r)
        {
          return r.name == ctrl;
        }) == running.end())
          {
            CNR_WARN(*m_logger, "CTRL " << ctrl << " of the HW" << hw << " is not running! ");
            return false;
          }
        }
      }
    }
  }
  catch (std::exception& e)
  {
    CNR_RETURN_FATAL(*m_logger, "Exception while checking the status of the Hw. Exception: " + std::string(e.what()));
  }
  catch (...)
  {
    CNR_FATAL(*m_logger, "Unhandled Exception while checking the status of the Hw. ");
    return false;
  }
  return true;
}

bool ConfigurationManager::checkRobotHwState(const std::string& hw, cnr_hardware_interface::StatusHw target)
{
  cnr_hardware_interface::StatusHw hw_status;
  ros::NodeHandle n("/");
  std::string error;
  if (!cnr_hardware_interface::get_state(n, hw, hw_status, ros::Duration(10), error))
  {
    CNR_FATAL(*m_logger, "The HW " << hw << " has not any valid state: " << error);
    return false;
  }
  if (hw_status != target)
  {
    CNR_FATAL(*m_logger, "The status of the HW '" << hw << "' is " << cnr_hardware_interface::to_string(hw_status)
                      << " while it should be " << cnr_hardware_interface::to_string(target));
    return false;
  }
  return true;
}

bool ConfigurationManager::callback(ConfigurationStruct* next_configuration, const int &strictness, const ros::Duration& watchdog)
{
  CNR_TRACE_START(*m_logger);

  const std::vector<std::string>  hw_active_names = getHardwareInterfacesNames(m_active_configuration);
  const std::vector<std::string>  hw_next_names   = next_configuration
                                                  ?  getHardwareInterfacesNames(*next_configuration)
                                                  : std::vector<std::string>();
  std::vector<std::string>        hw_to_load_names;
  std::vector<std::string>        hw_to_unload_names;
  std::vector<std::string>        hw_names_from_nodelet;


  CNR_INFO(*m_logger, cnr_logger::BM() << ">>>>>>>>>>>> Configuring HW " << cnr_logger::RST());
  if (!m_conf_loader.listHw(hw_names_from_nodelet, watchdog))
  {
    CNR_RETURN_FALSE(*m_logger, "Error in getting the loaded hardware interfaces by the nodelet manager: " + m_conf_loader.error());
  }

  extract<std::string>(hw_next_names, hw_active_names, &hw_to_load_names, &hw_to_unload_names, nullptr);
  CNR_DEBUG(*m_logger, "HW NAMES - ACTIVE (CLASS)  : " << to_string(hw_active_names));
  CNR_DEBUG(*m_logger, "HW NAMES - ACTIVE (NODELET): " << to_string(hw_names_from_nodelet));
  CNR_DEBUG(*m_logger, "HW NAMES - NEXT            : " << to_string(hw_next_names));
  CNR_DEBUG(*m_logger, "HW NAMES - TO LOAD         : " << to_string(hw_to_load_names));
  CNR_DEBUG(*m_logger, "HW NAMES - TO UNLOAD       : " << to_string(hw_to_unload_names));

  CNR_INFO(*m_logger, "Check coherence between nodelet status and configuration manager status");
  if (!equal(hw_active_names, hw_names_from_nodelet))
  {
    CNR_WARN(*m_logger, "Active configuration and the nodelet status is different. We force the unload of all the nodelet.. cross the fingers");
    if (!m_conf_loader.purgeHw(watchdog))
    {
      CNR_RETURN_FALSE(*m_logger, "The purge of the nodelets failed: " + m_conf_loader.error());
    }
    hw_to_load_names.insert(hw_to_load_names.begin(), hw_active_names.begin(), hw_active_names.end());
  }

  CNR_INFO(*m_logger, "Load the needed hardware interfaces by nodelets:" << to_string(hw_to_load_names, ""));
  for (const auto & hw_to_load_name : hw_to_load_names)
  {
    if (!m_conf_loader.loadHw(hw_to_load_name, watchdog, true))
    {
      CNR_RETURN_FALSE(*m_logger, "Loading of the RobotHW '" + hw_to_load_name + "' failed. Error:\n\t=>" + m_conf_loader.error());
    }

    if (!checkRobotHwState(hw_to_load_name))
    {
      CNR_RETURN_FALSE(*m_logger);
    }
  }
  CNR_INFO(*m_logger, cnr_logger::BM() << "<<<<<<<<<<<< Configuring HW "
                   << cnr_logger::BG() << "[ DONE ]" << cnr_logger::RST());


  CNR_INFO(*m_logger, cnr_logger::BM() << ">>>>>>>>>>>> Load and Start Controllers (hw: "
           << cnr_controller_interface::to_string(hw_next_names) << ")"  << cnr_logger::RST());

  m_conf_loader.loadAndStartControllers(hw_next_names, next_configuration, strictness);

  CNR_INFO(*m_logger, cnr_logger::BM() << "<<<<<<<<<<<< Load and Start Controllers "
                   << cnr_logger::BG() << "[ DONE ]" << cnr_logger::RST());


  CNR_INFO(*m_logger, cnr_logger::BM() << ">>>>>>>>>>>> Unload and Stop Controllers (hw: "
                   << cnr_controller_interface::to_string(hw_to_unload_names) << ")" << cnr_logger::RST() );
  if (!m_conf_loader.stopAndUnloadAllControllers(hw_to_unload_names))
  {
    CNR_INFO(*m_logger, cnr_logger::BM() << "<<<<<<<<<<<< Unload and Stop Controllers unload "
                     << cnr_logger::RED() << "FAILED" << cnr_logger::RST());
    CNR_RETURN_FALSE(*m_logger);
  }
  CNR_INFO(*m_logger, cnr_logger::BM() << "<<<<<<<<<<<< Unload and Stop Controllers unload "
                   << cnr_logger::BG() << "DONE" << cnr_logger::RST());


  CNR_INFO(*m_logger,  cnr_logger::BM() <<  ">>>>>>>>>>>> Unload unnecessary hw (" << to_string(hw_to_unload_names)
                       << ")" << cnr_logger::RST());
  if (!m_conf_loader.unloadHw(hw_to_unload_names, watchdog))
  {
    CNR_RETURN_FALSE(*m_logger, "Unload the configuration failed. Error: " + m_conf_loader.error());
  }
  CNR_INFO(*m_logger,  cnr_logger::BM() <<  "<<<<<<<<<<<< Unload unnecessary hw [DONE]" << cnr_logger::RST());


  CNR_DEBUG(*m_logger, "It is finished.");
  CNR_RETURN_TRUE(*m_logger);
}

bool ConfigurationManager::updateConfigurations()
{
  CNR_TRACE_START(*m_logger);

  std::map<std::string, ConfigurationStruct > configurations;
  XmlRpc::XmlRpcValue                         configuration_components;
  if (!m_nh.getParam("control_configurations", configuration_components))
  {
    std::string error = "Param '" + m_nh.getNamespace() + "/control_configurations' is not found." ;
    CNR_RETURN_BOOL(*m_logger, false, error);
  }

  std::string error;
  if (!param::get_configuration_components(configuration_components, configurations, error))
  {
    error = "Param '" + m_nh.getNamespace() + "/control_configurations' error: " + m_conf_loader.error() ;
    CNR_RETURN_BOOL(*m_logger, false, error);
  }

  m_configurations.clear();
  m_configurations = configurations;
  CNR_DEBUG(*m_logger, "CONFIGURATIONS AS IN ROSPARAM SERVER:\n" << to_string(configurations));
  CNR_RETURN_BOOL(*m_logger, true);
}

}  // namespace cnr_configuration_manager
