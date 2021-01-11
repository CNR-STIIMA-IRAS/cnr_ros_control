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
#pragma once // workaround clang-tidy qtcreator

#ifndef CNR_CONTOLLER_INTERFACE__CNR_JOINT_CONTROLLER_INTERFACE_IMPL_H
#define CNR_CONTOLLER_INTERFACE__CNR_JOINT_CONTROLLER_INTERFACE_IMPL_H

#include <sstream>
#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>
#include <rosdyn_utilities/chain_state.h>
#include <cnr_controller_interface/internal/cnr_handles.h>
#include <cnr_controller_interface/cnr_joint_controller_interface.h>
#include <rosdyn_core/primitives.h>
#include <rosdyn_utilities/chain_state.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace cnr
{
namespace control
{

template<int N,int MaxN,class H,class T>
JointController<N,MaxN,H,T>::~JointController()
{
  m_stop_update_transformations = true;
  if(m_update_transformations.joinable())
    m_update_transformations.join();
  CNR_TRACE_START(this->m_logger);
}

template<int N,int MaxN,class H,class T>
bool JointController<N,MaxN,H,T>::doInit()
{
  return true;
}
template<int N,int MaxN,class H,class T>
bool JointController<N,MaxN,H,T>::doStarting(const ros::Time& /*time*/)
{
  return true;
}

template<int N,int MaxN,class H,class T>
bool JointController<N,MaxN,H,T>::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  return true;
}

template<int N,int MaxN,class H,class T>
bool JointController<N,MaxN,H,T>::doStopping(const ros::Time& /*time*/)
{
  m_stop_update_transformations = true;
  if(m_update_transformations.joinable())
    m_update_transformations.join();
  return true;
}

template<int N,int MaxN,class H,class T>
bool JointController<N,MaxN,H,T>::doWaiting(const ros::Time& /*time*/)
{
  return true;
}

template<int N,int MaxN,class H,class T>
bool JointController<N,MaxN,H,T>::doAborting(const ros::Time& /*time*/)
{
  return true;
}

template<int N,int MaxN,class H,class T>
bool JointController<N,MaxN,H,T>::enterInit()
{
  m_cfrmt = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
  CNR_TRACE_START(this->m_logger);
  if(!Controller<T>::enterInit())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  std::vector<std::string> joint_names;
  rosdyn::get_joint_names(this->getControllerNh(),joint_names);
  if(joint_names.size()==1 && joint_names.front() == "all")
  {
    rosdyn::get_joint_names(this->getRootNh(), joint_names);
  }

  if(joint_names.size()==0)
  {
    CNR_RETURN_FALSE(this->m_logger, "Neither '" +  this->getControllerNamespace() + "/controlled_joint(s)' nor '"
                        + this->getControllerNamespace() + "/controlled_resources(s)' are specified. Abort" );
  }

  std::string base_link;
  if(!this->getControllerNh().getParam("base_link", base_link ) )
  {
    if(!this->getRootNh().getParam("base_link", base_link ) )
    {
      CNR_RETURN_FALSE(this->m_logger, "'Neither '" + this->getControllerNamespace() + "/base_link' " +
                  "nor '"      + this->getRootNamespace() + "/base_link' are not in rosparam server.");
    }
  }

  std::string tool_link;
  if(!this->getControllerNh().getParam("tool_link", tool_link ) )
  {
    if(!this->getRootNh().getParam("tool_link", tool_link ) )
    {
      CNR_RETURN_FALSE(this->m_logger, "'Neither '" + this->getControllerNamespace() + "/tool_link' " +
                "nor '"      + this->getRootNamespace() + "/tool_link' are not in rosparam server.");
    }
  }

  m_rkin.reset(new rosdyn::ChainInterface());
  std::stringstream report;
  int res = m_rkin->init(Controller<T>::getControllerNh(),joint_names, base_link, tool_link, report);
  if(res==1)
  {
    CNR_INFO_COND(this->m_logger,(report.str().length()>0), report.str() );
  }
  else if(res==0)
  {
    CNR_WARN_COND(this->m_logger, (report.str().length()>0), report.str() );
    CNR_WARN_COND(this->m_logger, (report.str().length()>0),
                  "Further tentative, looking parameters under: " << Controller<T>::getRootNamespace());
    report.str(std::string());
    if(m_rkin->init(Controller<T>::getRootNh(),joint_names, base_link, tool_link, report) != 1)
    {
      CNR_ERROR_COND(this->m_logger, (report.str().length()>0), report.str() );
      std::string result;
      if(ros::param::search("/", "robot_description_planning", result))
      {
        CNR_WARN(this->m_logger, " 'robot_description_planning' has been found under: " << result );
      }
      CNR_RETURN_FALSE(this->m_logger);
    }
  }
  else
  {
    CNR_ERROR_COND(this->m_logger, (report.str().length()>0), report.str() );
    CNR_RETURN_FALSE(this->m_logger);
  }


  m_rstate.init(m_rkin);
  for (unsigned int iAx=0; iAx<m_rkin->nAx(); iAx++)
  {
    try
    {
      m_handler.handles_[m_rkin->jointName(iAx)] = Controller<T>::m_hw->getHandle(m_rkin->jointName(iAx));
    }
    catch (...)
    {
      CNR_RETURN_FALSE(this->m_logger,
        "Controller '" + Controller<T>::getControllerNamespace() + "' failed in init. " + std::string("")
        + "The controlled joint named '" + m_rkin->jointName(iAx) + "' is not managed by hardware_interface");
    }
    CNR_DEBUG(this->m_logger,
      "Controller '" + Controller<T>::getControllerNamespace() + std::string("'")
      + "The controlled joint named '" + m_rkin->jointName(iAx) + "' is managed by hardware_interface");
  }

  CNR_RETURN_TRUE(this->m_logger);
}

template<int N,int MaxN,class H,class T>
bool JointController<N,MaxN,H,T>::enterStarting()
{
  CNR_TRACE_START(this->m_logger);
  if(!Controller<T>::enterStarting())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }
  m_handler >> m_rstate;

  m_stop_update_transformations = false;
  m_update_transformations = std::thread(&JointController<N,MaxN,H,T>::updateTransformations, this);

  CNR_RETURN_TRUE(this->m_logger);
}

template<int N,int MaxN,class H,class T>
bool JointController<N,MaxN,H,T>::enterUpdate()
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->m_logger);
  if(!Controller<T>::enterUpdate())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  std::lock_guard<std::mutex> lock(m_mtx);
  this->m_handler >> m_rstate;
  // NOTE: the transformations may take time, especially due the pseudo inversion of the Jacobian, to estimate the external wrench.
  // Therefore, they are executed in parallel
  //m_rstate.updateTransformations();

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->m_logger);
}

template<int N,int MaxN,class H,class T>
bool JointController<N,MaxN,H,T>::updateTransformations()
{
  auto q = m_rstate.q();
  auto qd = m_rstate.qd();
  auto qdd = m_rstate.qdd();
  auto external_effort = m_rstate.external_effort();

  CNR_TRACE_START_THROTTLE_DEFAULT(this->m_logger);
  ros::Rate rt(1.0/this->m_sampling_period);
  while(!m_stop_update_transformations && ros::ok())
  {
    {
      std::lock_guard<std::mutex> lock(m_mtx);
      q = m_rstate.q();
      qd = m_rstate.qd();
      qdd = m_rstate.qdd();
      external_effort = m_rstate.external_effort();
    }

    m_rstate.updateTransformations(&q, &qd, &qdd, &external_effort);

    rt.sleep();
  }

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->m_logger);
}

}  // cnr_controller_interface
}  // namespace cnr
#endif
