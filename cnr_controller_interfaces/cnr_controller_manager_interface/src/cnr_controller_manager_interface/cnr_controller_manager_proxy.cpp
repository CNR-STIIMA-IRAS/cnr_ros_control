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
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include <string>
#include <thread>

#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_controller_manager_interface/cnr_controller_manager_proxy.h>

namespace cnr_controller_manager_interface
{


/**
 * @brief ControllerManagerProxy::ControllerManagerProxy
 * @param nh
 */
ControllerManagerProxy::ControllerManagerProxy(std::shared_ptr<cnr_logger::TraceLogger>  logger,
                                               const std::string&                        hw_name,
                                               hardware_interface::RobotHW*              robot_hw,
                                               const ros::NodeHandle&                    nh)
: ControllerManager(logger,hw_name,robot_hw,nh)
{
  load_     = nh_.advertiseService("/" + hw_name + "/controller_manager_proxy/load_controller",
                                   &ControllerManagerProxy::loadControllerSrv, this);
  unload_   = nh_.advertiseService("/" + hw_name + "/controller_manager_proxy/unload_controller",
                                   &ControllerManagerProxy::unloadControllerSrv, this);
  doswitch_ = nh_.advertiseService("/" + hw_name + "/controller_manager_proxy/switch_controller",
                                   &ControllerManagerProxy::switchControllerSrv,this);
}

/**
 * @brief ControllerManagerProxy::loadController
 * @param req
 * @param res
 * @return
 */
bool ControllerManagerProxy::loadControllerSrv(controller_manager_msgs::LoadController::Request& req,
                                               controller_manager_msgs::LoadController::Response& res )
{
  CNR_TRACE_START(*logger_, "HW: "+ getHwName());
  std::lock_guard<std::mutex> lock(mtx_);
  res.ok = cm_.loadController(req.name);
  if( res.ok )
  {
    controllers_.emplace( req.name, cm_.getControllerByName(req.name) );
  }
  CNR_RETURN_TRUE(*logger_, "HW: " + getHwName());
}

/**
 * @brief ControllerManagerProxy::unloadController
 * @param req
 * @param res
 * @return
 */
bool ControllerManagerProxy::unloadControllerSrv(controller_manager_msgs::UnloadController::Request& req,
                                                 controller_manager_msgs::UnloadController::Response& res )
{
  CNR_TRACE_START(*logger_, "HW: "+ getHwName());
  std::lock_guard<std::mutex> lock(mtx_);
  res.ok = cm_.unloadController(req.name);
  if( res.ok )
  {
    if(controllers_.find(req.name) != controllers_.end())
    {
      controllers_.erase( controllers_.find(req.name) );
    }
  }
  CNR_RETURN_TRUE(*logger_, "HW: " + getHwName());
}

/**
 * @brief ControllerManagerProxy::switchController
 * @param req
 * @param res
 * @return
 */
bool ControllerManagerProxy::switchControllerSrv(controller_manager_msgs::SwitchController::Request& req,
                                                 controller_manager_msgs::SwitchController::Response& res )
{
  CNR_TRACE_START(*logger_, "HW: "+ getHwName());
  std::lock_guard<std::mutex> lock(mtx_);
  for ( const auto ctrl : req.start_controllers )
  {
    if( controllers_.find(ctrl) == controllers_.end() )
    {
      controller_manager_msgs::LoadController load;
      load.request.name = ctrl;
      if ( !loadControllerSrv(load.request,load.response))
      {
        CNR_RETURN_FALSE(*logger_, "Wierd Error in loading the '" + ctrl + "'" );
      }
      if( !load.response.ok )
      {
        CNR_RETURN_FALSE(*logger_, "Failed in loading the '" + ctrl + "'" );
      }
    }
  }
  res.ok = cm_.switchController( req.start_controllers, req.stop_controllers, req.strictness);
  CNR_RETURN_TRUE(*logger_, "HW: " + getHwName());
}

/**
 * @brief ControllerManagerProxy::diagnostics
 * @param stat
 */
void ControllerManagerProxy::diagnosticsInfo(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for ( const auto & ctrl : controllers_)
  {
    cnr_controller_interface::ControllerDiagnostic* cnr_ctrl =
                                    dynamic_cast<cnr_controller_interface::ControllerDiagnostic*>(ctrl.second);
    if( cnr_ctrl )
    {
      cnr_ctrl->diagnosticsInfo(stat);
    }
  }
}


/**
 * @brief ControllerManagerProxy::diagnostics
 * @param stat
 */
void ControllerManagerProxy::diagnosticsWarn(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for ( const auto & ctrl : controllers_)
  {
    cnr_controller_interface::ControllerDiagnostic* cnr_ctrl =
                                    dynamic_cast<cnr_controller_interface::ControllerDiagnostic*>(ctrl.second);
    if( cnr_ctrl )
    {
      cnr_ctrl->diagnosticsWarn(stat);
    }
  }
}


/**
 * @brief ControllerManagerProxy::diagnostics
 * @param stat
 */
void ControllerManagerProxy::diagnosticsError(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for ( const auto & ctrl : controllers_)
  {
    cnr_controller_interface::ControllerDiagnostic* cnr_ctrl =
                                    dynamic_cast<cnr_controller_interface::ControllerDiagnostic*>(ctrl.second);
    if( cnr_ctrl )
    {
      cnr_ctrl->diagnosticsError(stat);
    }
  }
}

void ControllerManagerProxy::diagnosticsPerformance(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for ( const auto & ctrl : controllers_)
  {
    cnr_controller_interface::ControllerDiagnostic* cnr_ctrl =
                                    dynamic_cast<cnr_controller_interface::ControllerDiagnostic*>(ctrl.second);
    if( cnr_ctrl )
    {
      cnr_ctrl->diagnosticsPerformance(stat);
    }
  }
}



}

