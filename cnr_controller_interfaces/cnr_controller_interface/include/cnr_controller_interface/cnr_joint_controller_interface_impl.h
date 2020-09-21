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
#include <cnr_controller_interface/cnr_joint_controller_interface.h>
#include <rosdyn_core/primitives.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace cnr_controller_interface
{

template< class T >
JointController< T >::~JointController()
{
  m_kin.reset();
  m_state.reset();
  CNR_TRACE_START(*cnr_controller_interface::Controller< T >::m_logger);
}

template< class T >
bool JointController< T >::doInit()
{
  return true;
}
template< class T >
bool JointController<T>::doStarting(const ros::Time& /*time*/)
{
  return true;
}

template< class T >
bool JointController<T>::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  return true;
}

template< class T >
bool JointController<T>::doStopping(const ros::Time& /*time*/)
{
  return true;
}

template< class T >
bool JointController<T>::doWaiting(const ros::Time& /*time*/)
{
  return true;
}

template< class T >
bool JointController<T>::doAborting(const ros::Time& /*time*/)
{
  return true;
}

template< class T >
bool JointController<T>::enterInit()
{
  m_cfrmt = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
  CNR_TRACE_START(*Controller<T>::m_logger);
  if (!Controller<T>::enterInit())
  {
    CNR_RETURN_FALSE(*Controller<T>::m_logger);
  }
  m_kin.reset(new KinematicsStruct());
  m_kin->init(Controller<T>::m_logger, Controller<T>::getRootNh(), Controller<T>::getControllerNh());

  m_state.reset(new KinematicStatus());
  m_state->resize(m_kin->nAx());
 for (unsigned int iAx = 0; iAx < nAx(); iAx++)
  {
    try
    {
      Controller<T>::m_hw->getHandle(m_kin->jointName(iAx));
    }
    catch (...)
    {
      CNR_RETURN_FALSE(*Controller<T>::m_logger,
        "Controller '" + Controller<T>::getControllerNamespace() + "' failed in init. " + std::string("")
        + "The controlled joint named '" + m_kin->jointName(iAx) + "' is not managed by hardware_interface");
    }
  }

  CNR_RETURN_BOOL(*Controller<T>::m_logger, Controller<T>::dump_state());
}

template< class T >
bool JointController<T>::enterStarting()
{
  CNR_TRACE_START(*Controller<T>::m_logger);
  if (!Controller<T>::enterStarting())
  {
    CNR_RETURN_FALSE(*Controller<T>::m_logger);
  }
  extract< T >( Controller<T>::m_hw, m_state);
  m_state->qd.setZero();
  m_state->qdd.setZero();
  m_state->effort.setZero();

  m_kin->updateTransformation(*m_state);

  CNR_RETURN_TRUE(*Controller<T>::m_logger);
}

template< class T >
bool JointController<T>::enterUpdate()
{
  CNR_TRACE_START_THROTTLE(*Controller<T>::m_logger, 20.0);
  if (!Controller<T>::enterUpdate())
  {
    CNR_RETURN_FALSE(*Controller<T>::m_logger);
  }
  extract< T >( Controller<T>::m_hw, m_state);
  m_kin->updateTransformation(*m_state);

  CNR_RETURN_TRUE_THROTTLE(*Controller<T>::m_logger, 20.0);
}




} // cnr_controller_interface
#endif
