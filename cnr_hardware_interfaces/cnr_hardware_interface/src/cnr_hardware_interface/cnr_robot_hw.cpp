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

#include <algorithm>
#include <list>
#include <vector>
#include <map>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_hardware_interface/cnr_robot_hw.h>

#define GENERATE_ENUM_STRINGS  // Start string generation
#include <cnr_hardware_interface/internal/cnr_robot_hw_status.h>
#undef GENERATE_ENUM_STRINGS   // Stop string generation

namespace cnr_hardware_interface
{

RobotHW::RobotHW()
  : m_status(cnr_hardware_interface::CREATED), m_shutted_down(false), m_is_first_read(true), m_set_status_param(nullptr)
{
//  if ( m_resource_names.size() == 0 )
//  {
//    dump_state( cnr_hardware_interface::ERROR );
//  }
  dump_state(cnr_hardware_interface::CREATED);
}

RobotHW::~RobotHW()
{
  CNR_TRACE_START(*m_logger);
  if (!m_shutted_down)
  {
    if (!shutdown())
    {
      dump_state(cnr_hardware_interface::ERROR);
    }
    else
    {
      dump_state(cnr_hardware_interface::SHUTDOWN);
    }
  }
  CNR_TRACE(*m_logger, "[  DONE] ");
}

bool RobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robothw_nh)
{
  std::string n = "HW" + robothw_nh.getNamespace();
  std::replace(n.begin(), n.end(), '/', '_');

  m_logger.reset(new cnr_logger::TraceLogger(n, robothw_nh.getNamespace()));
  CNR_TRACE_START(*m_logger);
  if (enterInit(root_nh, robothw_nh)  && doInit() && exitInit())
  {
    CNR_RETURN_TRUE(*m_logger, "RobotHW '" + m_robot_name + "' Initialization OK");
  }


  CNR_RETURN_FALSE(*m_logger,  "RobotHW '" + m_robot_name + "' Initialization Failed");
}

void RobotHW::read(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE(*m_logger, 5.0);

  m_robot_hw_queue.callAvailable();

  if (!doRead(time, period))
  {
    dump_state(cnr_hardware_interface::ERROR);
    CNR_RETURN_NOTOK_THROTTLE(*m_logger, void(), 5.0);
  }

  if (m_is_first_read)
  {
    dump_state(cnr_hardware_interface::RUNNING);
    m_is_first_read = false;
    if (m_set_status_param)
    {
      m_set_status_param("first_configuration");
    }
  }

  CNR_RETURN_OK_THROTTLE(*m_logger, void(), 5.0);
}

void RobotHW::write(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE(*m_logger, 5.0);
  if (enterWrite() && doWrite(time, period) && exitWrite())
  {
    CNR_RETURN_OK_THROTTLE(*m_logger, void(), 5.0);
  }

  dump_state(cnr_hardware_interface::ERROR);
  CNR_RETURN_NOTOK_THROTTLE(*m_logger, void(), 5.0, "Error in writing...");
}

bool RobotHW::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
                            const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  CNR_TRACE_START(*m_logger);
  if (enterPrepareSwitch(start_list, stop_list) && doPrepareSwitch(start_list, stop_list) && exitPrepareSwitch())
  {
    CNR_RETURN_TRUE(*m_logger);
  }

  m_is_first_read = false;
  CNR_RETURN_FALSE(*m_logger);
}

bool RobotHW::checkForConflict(const std::list< hardware_interface::ControllerInfo >& info) const
{
  CNR_TRACE_START(*m_logger);
  if (::hardware_interface::RobotHW::checkForConflict(info))
  {
    CNR_RETURN_TRUE(*m_logger, "Base Check failed");
  }
  if (enterCheckForConflict(info) || doCheckForConflict(info) || exitCheckForConflict())
  {
    CNR_RETURN_TRUE(*m_logger);
  }

  CNR_RETURN_FALSE(*m_logger);
}

bool RobotHW::shutdown()
{
  CNR_TRACE_START(*m_logger, ">>>> RobotHW Shutdown (" + m_robot_name + ")");
  if (enterShutdown()  && doShutdown() && exitShutdown())
  {
    CNR_RETURN_TRUE(*m_logger, "<<<< Robot Shutdown (" + m_robot_name + ")");
  }
  dump_state(cnr_hardware_interface::ERROR);
  CNR_RETURN_FALSE(*m_logger, "<<<< Robot Shutdown Failure (" + m_robot_name + ")");
}

bool RobotHW::enterShutdown()
{
  CNR_TRACE_START(*m_logger);
  CNR_RETURN_TRUE(*m_logger);
}

bool RobotHW::exitShutdown()
{
  CNR_TRACE_START(*m_logger);
  if (m_set_status_param)
  {
    m_set_status_param("shutdown_configuration");
  }
  m_shutted_down =  true;
  bool ret = dump_state(cnr_hardware_interface::SHUTDOWN);
  CNR_RETURN_BOOL(*m_logger, ret);
}

bool RobotHW::enterWrite()
{
  return true;
}

bool RobotHW::exitWrite()
{
  if (m_status_history.back() != cnr_hardware_interface::to_string(m_status))
  {
    if (m_set_status_param)
    {
      m_set_status_param("last_valid_configuration");
    }
    m_status_history.push_back(cnr_hardware_interface::to_string(m_status));
  }

  return dump_state();
}

bool RobotHW::enterInit(ros::NodeHandle& root_nh, ros::NodeHandle& robothw_nh)
{
  CNR_TRACE_START(*m_logger);
  m_robothw_nh  = robothw_nh;
  m_robothw_nh.setCallbackQueue(&m_robot_hw_queue);
  m_root_nh     = root_nh;
  m_stop_thread = false;
  m_robot_name  = extractRobotName(m_robothw_nh.getNamespace());
  m_set_param   = m_robothw_nh.advertiseService("writeParams", &cnr_hardware_interface::RobotHW::setParamServer, this);
  m_get_param   = m_robothw_nh.advertiseService("readParams", &cnr_hardware_interface::RobotHW::getParamServer, this);

  m_robot_hw_queue.callAvailable();

  CNR_RETURN_TRUE(*m_logger);
}

bool RobotHW::exitInit()
{
  CNR_TRACE_START(*m_logger);
  bool ret = (m_resource_names.size() > 0);

  if (!ret)
  {
    CNR_FATAL(*m_logger, "Reources names not set! Remeber to assign them in the doInit function. ");
  }

  dump_state(ret ? cnr_hardware_interface::INITIALIZED : cnr_hardware_interface::ERROR);
  CNR_RETURN_BOOL(*m_logger, ret);
}

bool RobotHW::enterPrepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
                                 const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  CNR_TRACE_START(*m_logger);
  for (const hardware_interface::ControllerInfo& ctrl : stop_list)
  {
    std::list<hardware_interface::ControllerInfo>::iterator stopped_controller =
      std::find_if(m_active_controllers.begin(), m_active_controllers.end(),
                   [&ctrl](auto & it)
    {
      return (it.name == ctrl.name);
    });  //NOLINT

    if (m_active_controllers.end() == stopped_controller)
    {
      add_diagnostic_message("ERROR", "controller '" + ctrl.name + "' is not active, so I cannot stop it",
                              { {"Transition", "switching"} }, true);  //NOLINT
      dump_state(cnr_hardware_interface::CTRL_ERROR);
      CNR_RETURN_FALSE(*m_logger);
    }
    m_active_controllers.erase(stopped_controller);
  }
  for (const hardware_interface::ControllerInfo& ctrl : start_list)
  {
    std::list<hardware_interface::ControllerInfo>::iterator stopped_controller =
      std::find_if(m_active_controllers.begin(), m_active_controllers.end(),
                   [&ctrl](auto & it)
    {
      return (it.name == ctrl.name);
    });  //NOLINT
    if (m_active_controllers.end() != stopped_controller)
    {
      CNR_WARN(*m_logger, "controller " << ctrl.name << "is not active, so I cannot stop it");
    }
    else
    {
      m_active_controllers.push_back(ctrl);
    }
  }
  CNR_RETURN_TRUE(*m_logger);
}

bool RobotHW::exitPrepareSwitch()
{
  return dump_state(cnr_hardware_interface::READY);
}

bool RobotHW::enterCheckForConflict(const std::list< hardware_interface::ControllerInfo >& info) const
{
  // Each controller can use more than a hardware_interface for a single joint (for example: position, velocity, effort)
  // One controller can control more than one joint.
  // A joint can be used only by a controller.
  std::vector<bool> global_resource_used(m_resource_names.size());
  std::fill(global_resource_used.begin(), global_resource_used.end(), false);

  for (hardware_interface::ControllerInfo controller : info)
  {
    std::vector<bool> single_controller_resource_used(m_resource_names.size());
    std::fill(single_controller_resource_used.begin(), single_controller_resource_used.end(), false);

    for (hardware_interface::InterfaceResources res : controller.claimed_resources)
    {
      for (std::string name : res.resources)
      {
        for (unsigned int iJ = 0; iJ < m_resource_names.size(); iJ++)
        {
          if (!name.compare(m_resource_names.at(iJ)))
          {
            if (global_resource_used.at(iJ))   // if already used by another
            {
              CNR_FATAL(*m_logger, "Joint " + name + "%s is already used by another controller" );  //NOLINT
              dump_state(cnr_hardware_interface::CTRL_ERROR);
              return true;
            }
            else
            {
              single_controller_resource_used.at(iJ) = true;
            }
          }
        }
      }
    }
    for (unsigned int iJ = 0; iJ < m_resource_names.size() ; iJ++)
    {
      global_resource_used.at(iJ) = global_resource_used.at(iJ) || single_controller_resource_used.at(iJ);
    }
  }
  return false;
}

bool RobotHW::getParamServer(configuration_msgs::GetConfigRequest& req, configuration_msgs::GetConfigResponse& res)
{
  return true;
}

bool RobotHW::setParamServer(configuration_msgs::SetConfigRequest& req, configuration_msgs::SetConfigResponse& res)
{
  return true;
}

void RobotHW::add_diagnostic_message(const std::string& level
                                     , const std::string& summary
                                     , const std::map<std::string, std::string>& key_values
                                     , const bool verbose)
{
  diagnostic_msgs::DiagnosticStatus diag;
  diag.name        = m_robot_name;
  diag.hardware_id = m_robot_name;
  diag.message     = summary;

  if (level == "OK")
  {
    diag.level = diagnostic_msgs::DiagnosticStatus::OK;
    CNR_INFO_COND(*m_logger, verbose, "[" << m_robot_name << " ] " << summary);
  }
  if (level == "WARN")
  {
    diag.level = diagnostic_msgs::DiagnosticStatus::WARN;
    CNR_WARN_COND(*m_logger, verbose, "[" << m_robot_name << " ] " << summary);
  }
  if (level == "ERROR")
  {
    diag.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    CNR_ERROR_COND(*m_logger, verbose, "[" << m_robot_name << " ] " << summary);
  }
  if (level == "STALE")
  {
    diag.level = diagnostic_msgs::DiagnosticStatus::STALE;
    CNR_INFO_COND(*m_logger, verbose, "[" << m_robot_name << " ] " << summary);
  }

  for (const auto & key_value : key_values)
  {
    diagnostic_msgs::KeyValue kv;
    kv.key = key_value.first;
    kv.value = key_value.second;
    diag.values.push_back(kv);
  }
  std::lock_guard<std::mutex> lock(m_mutex);
  m_diagnostic.status.push_back(diag);
}


void RobotHW::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat, int level)
{
  boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();

  std::lock_guard<std::mutex> lock(m_mutex);
  stat.hardware_id = m_robot_name;
  stat.name        = "RobotHW ["
                   + ( level == (int)diagnostic_msgs::DiagnosticStatus::OK  ? std::string("Info")
                     : level == (int)diagnostic_msgs::DiagnosticStatus::WARN ? std::string("Warn")
                     : std::string("Error") )
                   +"]";
  bool something_to_add = false;
  for (  const diagnostic_msgs::DiagnosticStatus & s : m_diagnostic.status )
  {
    something_to_add |= static_cast<int>( s.level ) == level;
  }
  if ( something_to_add )
  {
    stat.level       = level == (int)diagnostic_msgs::DiagnosticStatus::OK ? diagnostic_msgs::DiagnosticStatus::OK
                     : level == (int)diagnostic_msgs::DiagnosticStatus::WARN ? diagnostic_msgs::DiagnosticStatus::WARN
                     : level == (int)diagnostic_msgs::DiagnosticStatus::ERROR ? diagnostic_msgs::DiagnosticStatus::ERROR
                     : diagnostic_msgs::DiagnosticStatus::STALE;
    stat.message     = "Log of the status [" + boost::posix_time::to_iso_string(my_posix_time) + "]";
    for ( const diagnostic_msgs::DiagnosticStatus & s : m_diagnostic.status )
    {
      diagnostic_msgs::KeyValue k;
      k.key = s.name;
      k.value = s.message;
      stat.add(k.key, k.value);
    }
    m_diagnostic.status.erase(
        std::remove_if(
            m_diagnostic.status.begin(),
            m_diagnostic.status.end(),
            [&](diagnostic_msgs::DiagnosticStatus const & p) { return p.level == level; }
        ),
        m_diagnostic.status.end()
    );
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "None Error in the queue ["
                 + boost::posix_time::to_iso_string(my_posix_time) + "]");
  }


}

void RobotHW::diagnosticsInfo(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  diagnostics(stat,diagnostic_msgs::DiagnosticStatus::OK);
}

void RobotHW::diagnosticsWarn(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  diagnostics(stat,diagnostic_msgs::DiagnosticStatus::WARN);
}

void RobotHW::diagnosticsError(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  diagnostics(stat,diagnostic_msgs::DiagnosticStatus::ERROR);
}

bool RobotHW::dump_state() const
{
  std::string last_status = cnr_hardware_interface::to_string(m_status);
  if (m_status_history.size() == 0)
  {
    m_status_history.push_back(last_status);
  }
  else
  {
    if (m_status_history.back() != last_status)
    {
      m_status_history.push_back(last_status);
      m_robothw_nh.setParam(last_status_param(m_robothw_nh.getNamespace()), last_status);
      m_robothw_nh.setParam(status_param(m_robothw_nh.getNamespace()), m_status_history);
      CNR_DEBUG(*m_logger, "RobotHW '" << m_robothw_nh.getNamespace()
                            << "' New Status " <<  cnr_hardware_interface::to_string(m_status));
    }
  }
  return true;
}

bool RobotHW::dump_state(const cnr_hardware_interface::StatusHw& status) const
{
  if ((m_status_history.size() == 0) || (m_status_history.back() != cnr_hardware_interface::to_string(m_status)))
  {
    m_status_history.push_back(cnr_hardware_interface::to_string(m_status));
  }
  m_status = status;
  return dump_state();
}


}  // namespace cnr_hardware_interface
