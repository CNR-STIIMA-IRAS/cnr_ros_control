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
#ifndef CNR_CONTOLLER_INTERFACE__CNR_JOINT_CONTROLLER_INTERFACE_IMPL_H
#define CNR_CONTOLLER_INTERFACE__CNR_JOINT_CONTROLLER_INTERFACE_IMPL_H

#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>
#include <cnr_controller_interface/internal/cnr_handles.h>
#include <cnr_controller_interface/cnr_joint_controller_interface.h>
#include <rosdyn_core/primitives.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace cnr_controller_interface
{

template<class H, class T>
JointController<H,T>::~JointController()
{
  m_rkin.reset();
  m_rstate.reset();
  CNR_TRACE_START(*cnr_controller_interface::Controller< T >::m_logger);
}

template<class H, class T>
bool JointController<H,T>::doInit()
{
  return true;
}
template<class H, class T>
bool JointController<H,T>::doStarting(const ros::Time& /*time*/)
{
  return true;
}

template<class H, class T>
bool JointController<H,T>::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  return true;
}

template<class H, class T>
bool JointController<H,T>::doStopping(const ros::Time& /*time*/)
{
  return true;
}

template<class H, class T>
bool JointController<H,T>::doWaiting(const ros::Time& /*time*/)
{
  return true;
}

template<class H, class T>
bool JointController<H,T>::doAborting(const ros::Time& /*time*/)
{
  return true;
}

template<class H, class T>
bool JointController<H,T>::enterInit()
{
  m_cfrmt = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
  CNR_TRACE_START(this->m_logger);
  if (!Controller<T>::enterInit())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }
  m_rkin.reset(new KinematicsStruct());
  m_rkin->init(Controller<T>::m_logger, Controller<T>::getRootNh(), Controller<T>::getControllerNh());

  m_rstate.reset(new KinematicStatus(m_rkin));
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

template<class H, class T>
bool JointController<H,T>::enterStarting()
{
  CNR_TRACE_START(this->m_logger);
  if (!Controller<T>::enterStarting())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }
  m_handler >> m_rstate;
  
  m_rstate->updateTransformation();

  CNR_RETURN_TRUE(this->m_logger);
}

template<class H, class T>
bool JointController<H,T>::enterUpdate()
{
  CNR_TRACE_START_THROTTLE(this->m_logger, 20.0);
  if (!Controller<T>::enterUpdate())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }
  this->m_handler >> m_rstate;
  m_rstate->updateTransformation();

  CNR_RETURN_TRUE_THROTTLE(this->m_logger, 20.0);
}




} // cnr_controller_interface
#endif
