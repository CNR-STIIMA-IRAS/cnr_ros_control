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
#include <cstring>
#include <atomic>
#include <string>
#include <thread>
#include <cnr_controller_interface_params/cnr_controller_interface_params.h>
#include <cnr_controller_manager_interface/cnr_controller_manager_proxy.h>

namespace cnr_controller_manager_interface
{


/**
 * @brief ControllerManagerProxy::ControllerManagerProxy
 * @param nh
 */
ControllerManagerProxy::ControllerManagerProxy(const cnr_logger::TraceLoggerPtr&  logger,
                                               const std::string&           hw_name,
                                               controller_manager::ControllerManager* cm)
: ControllerManagerInterface(logger,hw_name,cm)
{
  load_     = nh_.advertiseService("/" + hw_name + "/controller_manager_proxy/load_controller",
                                   &ControllerManagerProxy::loadControllerSrv, this);
  unload_   = nh_.advertiseService("/" + hw_name + "/controller_manager_proxy/unload_controller",
                                   &ControllerManagerProxy::unloadControllerSrv, this);
  doswitch_ = nh_.advertiseService("/" + hw_name + "/controller_manager_proxy/switch_controller",
                                   &ControllerManagerProxy::switchControllerSrv,this);
}

ControllerManagerProxy::~ControllerManagerProxy()
{
  load_.shutdown();
  unload_.shutdown();
  doswitch_.shutdown();
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
  CNR_TRACE_START(logger_, "HW: "+ getHwName());
  std::lock_guard<std::mutex> lock(mtx_);
  res.ok = this->loadController(req.name);
  if(!res.ok)
  {
    CNR_ERROR(logger_, "Error in loading '" + req.name+ "' (HW: " + getHwName()+")");
  }
  CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
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
  CNR_TRACE_START(logger_, "HW: "+ getHwName());
  std::lock_guard<std::mutex> lock(mtx_);
  res.ok = this->unloadController(req.name);
  if(!res.ok)
  {
    CNR_ERROR(logger_, "Error in unloading '" + req.name+ "' (HW: " + getHwName()+")");
  }
  CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
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
  CNR_TRACE_START(logger_, "HW: "+ getHwName());
  std::lock_guard<std::mutex> lock(mtx_);
  for ( const auto ctrl : req.start_controllers )
  {
    if( controllers_.find(ctrl) == controllers_.end() )
    {
      res.ok = this->loadController(ctrl);
      if(!res.ok)
      {
        CNR_ERROR(logger_, "Switching Srv failure: error in prelimniary loading '" 
                              + ctrl + "' (HW: " + getHwName()+")");
        CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
      }
    }
  }
  res.ok = this->switchController( req.start_controllers, req.stop_controllers, req.strictness);
  CNR_RETURN_TRUE(logger_, "HW: " + getHwName());
}


}  // namespace cnr_controller_manager_interface

