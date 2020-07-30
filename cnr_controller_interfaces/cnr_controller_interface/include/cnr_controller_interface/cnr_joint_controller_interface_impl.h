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
    CNR_TRACE_START(*Controller<T>::m_logger);
    if (!Controller<T>::enterInit())
    {
      CNR_RETURN_FALSE(*Controller<T>::m_logger);
    }

    XmlRpc::XmlRpcValue value;
    if (!Controller<T>::getControllerNh().getParam("controlled_joint", value)
    &&  !Controller<T>::getControllerNh().getParam("controlled_joints", value))
    {
      CNR_RETURN_FALSE(*Controller<T>::m_logger,
                       "The param " + Controller<T>::getControllerNamespace() + "/controlled_joint(s) not defined");
    }

    if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      if (value[0].getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        for (int i = 0; i < value.size(); i++)
        {
          m_joint_names.push_back((std::string)(value[i]));
        }
      }
      else
      {
        CNR_RETURN_FALSE(*Controller<T>::m_logger,
                         "The param " + Controller<T>::getControllerNamespace() + "/controlled_joint(s) bad formed");
      }
    }
    else if (value.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      m_joint_names.push_back((std::string)(value));
    }
    else
    {
      CNR_RETURN_FALSE(*Controller<T>::m_logger,
                       "The param " + Controller<T>::getControllerNamespace() + "/controlled_joint(s) bad formed");
    }

    m_nAx = m_joint_names.size();

    std::string robot_description_param;
    std::string robot_description;
    if (!Controller<T>::getControllerNh().getParam("robot_description_param", robot_description_param ) )
    {
      CNR_WARN(*Controller<T>::m_logger, Controller<T>::getControllerNamespace() + "/robot_description_param/ is not in rosparam server. Superimposed defualt value '/robot_description'");
      robot_description_param = "/robot_description";
    }
    if (!Controller<T>::getControllerNh().getParam(robot_description_param, robot_description))
    {
      CNR_FATAL(*Controller<T>::m_logger, "Parameter '/robot_description' does not exist");
      CNR_RETURN_FALSE(*Controller<T>::m_logger);
    }
    m_model = urdf::parseURDF(robot_description);

    m_state.resize(m_nAx);
    m_upper_limit    .resize(m_nAx); m_upper_limit  .setZero();
    m_lower_limit    .resize(m_nAx); m_lower_limit  .setZero();
    m_qd_limit       .resize(m_nAx); m_qd_limit     .setZero();
    m_qdd_limit      .resize(m_nAx); m_qdd_limit    .setZero();
    for (unsigned int iAx = 0; iAx < m_nAx; iAx++)
    {
      try
      {
          m_upper_limit(iAx) = m_model->getJoint(m_joint_names.at(iAx))->limits->upper;
          m_lower_limit(iAx) = m_model->getJoint(m_joint_names.at(iAx))->limits->lower;

          if ((m_upper_limit(iAx) == 0) && (m_lower_limit(iAx) == 0))
          {
            m_upper_limit(iAx) = std::numeric_limits<double>::infinity();
            m_lower_limit(iAx) = -std::numeric_limits<double>::infinity();
            ROS_INFO("upper and lower limits are both equal to 0, set +/- infinity");
          }

          bool has_velocity_limits;
          if (!Controller<T>::getControllerNh().getParam(
                "/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/has_velocity_limits", has_velocity_limits))
            has_velocity_limits = false;
          bool has_acceleration_limits;
          if (!Controller<T>::getControllerNh().getParam(
                "/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/has_acceleration_limits", has_acceleration_limits))
            has_acceleration_limits = false;

          m_qd_limit(iAx) = m_model->getJoint(m_joint_names.at(iAx))->limits->velocity;
          if (has_velocity_limits)
          {
            double vel;
            if (!Controller<T>::getControllerNh().getParam("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_velocity", vel))
            {
              ROS_ERROR_STREAM("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_velocity is not defined");
              return false;
            }
            if (vel < m_qd_limit(iAx))
              m_qd_limit(iAx) = vel;
          }

          if (has_acceleration_limits)
          {
            double acc;
            if (!Controller<T>::getControllerNh().getParam("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_acceleration", acc))
            {
              ROS_ERROR_STREAM("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_acceleration is not defined");
              return false;
            }
            m_qdd_limit(iAx) = acc;
          }
          else
            m_qdd_limit(iAx) = 10 * m_qd_limit(iAx);


        Controller<T>::m_hw->getHandle(m_joint_names.at(iAx));
      }
      catch (...)
      {
        CNR_RETURN_FALSE(*Controller<T>::m_logger,
          "Controller '" + Controller<T>::getControllerNamespace() + "' failed in init. " + std::string("")
          + "The controlled joint named '" + m_joint_names.at(iAx) + "' is not managed by hardware_interface");
      }
    }

    if (!Controller<T>::getControllerNh().getParam("base_link", m_base_link ) )
    {
      CNR_ERROR(*Controller<T>::m_logger, Controller<T>::getControllerNamespace() + "/base_link is not in rosparam server.");
      CNR_RETURN_FALSE(*Controller<T>::m_logger);
    }

    if (!Controller<T>::getControllerNh().getParam("tool_link", m_tool_link ) )
    {
      CNR_ERROR(*Controller<T>::m_logger, Controller<T>::getControllerNamespace() + "/tool_link is not in rosparam server.");
      CNR_RETURN_FALSE(*Controller<T>::m_logger);
    }

    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.806;

    std::shared_ptr<rosdyn::Link> root_link(new rosdyn::Link);  //link primitivo da cui parte la catena cinematica (world ad esempio)
    root_link->fromUrdf(m_model->root_link_);

    m_chain.reset(new rosdyn::Chain(root_link, m_base_link, m_tool_link, gravity)); //ricostruisce tutta la catena cinematica andando a leggere l'URDF
    m_chain->setInputJointsName(m_joint_names);

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
  m_state.qd.setZero();
  m_state.qdd.setZero();
  m_state.effort.setZero();

  m_Tbt = m_chain->getTransformation(m_state.q);

  CNR_RETURN_TRUE(*Controller<T>::m_logger);
}

template< class T >
bool JointController<T>::enterUpdate()
{
  CNR_TRACE_START(*Controller<T>::m_logger);
  if (!Controller<T>::enterStarting())
  {
    CNR_RETURN_FALSE(*Controller<T>::m_logger);
  }
  extract< T >( Controller<T>::m_hw, m_state);
  m_Tbt   = m_chain->getTransformation(m_state.q);
  m_J     = m_chain->getJacobian( m_state.q );
  m_twist = m_J * m_state.qd;

  CNR_RETURN_TRUE(*Controller<T>::m_logger);
}




} // cnr_controller_interface
#endif
