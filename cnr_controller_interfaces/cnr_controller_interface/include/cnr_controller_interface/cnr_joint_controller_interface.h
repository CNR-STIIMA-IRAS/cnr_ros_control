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
#ifndef __CNR__JOINT_CONTROLLER_INTERFACE__
#define __CNR__JOINT_CONTROLLER_INTERFACE__

#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_controller_interface/cnr_controller_interface.h>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>


namespace cnr_controller_interface
{

/**
 *
 *
 *
 * Base class to log the controller status
 */
template< class T >
class JointController: public cnr_controller_interface::Controller< T >
{
public:

  ~JointController()
  {
    CNR_TRACE_START(*cnr_controller_interface::Controller< T >::m_logger);
  }

  virtual bool doInit()
  {
    return true;
  }
  virtual bool doStarting(const ros::Time& /*time*/)
  {
    return true;
  }
  virtual bool doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
  {
    return true;
  }
  virtual bool doStopping(const ros::Time& /*time*/)
  {
    return true;
  }
  virtual bool doWaiting(const ros::Time& /*time*/)
  {
    return true;
  }
  virtual bool doAborting(const ros::Time& /*time*/)
  {
    return true;
  }

  const std::vector< std::string >&        getJointNames() const
  {
    return m_joint_names;
  }
  std::shared_ptr<cnr_logger::TraceLogger> logger()
  {
    return cnr_controller_interface::Controller< T >::m_logger;
  }



public:

  bool enterInit()
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
    std::string robot_description;
    if (!Controller<T>::getControllerNh().getParam("/robot_description", robot_description))
    {
      CNR_FATAL(*Controller<T>::m_logger, "Parameter '/robot_description' does not exist");
      CNR_RETURN_FALSE(*Controller<T>::m_logger);
    }
    m_model = urdf::parseURDF(robot_description);

    m_upper_limit.resize(m_nAx);
    m_lower_limit.resize(m_nAx);
    m_velocity_limit.resize(m_nAx);
    m_acceleration_limit.resize(m_nAx);


    for (unsigned int iAx = 0; iAx < m_nAx; iAx++)
    {
      try
      {
          m_upper_limit.at(iAx) = m_model->getJoint(m_joint_names.at(iAx))->limits->upper;
          m_lower_limit.at(iAx) = m_model->getJoint(m_joint_names.at(iAx))->limits->lower;

          if ((m_upper_limit.at(iAx) == 0) && (m_lower_limit.at(iAx) == 0))
          {
            m_upper_limit.at(iAx) = std::numeric_limits<double>::infinity();
            m_lower_limit.at(iAx) = -std::numeric_limits<double>::infinity();
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

          m_velocity_limit.at(iAx) = m_model->getJoint(m_joint_names.at(iAx))->limits->velocity;
          if (has_velocity_limits)
          {
            double vel;
            if (!Controller<T>::getControllerNh().getParam("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_velocity", vel))
            {
              ROS_ERROR_STREAM("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_velocity is not defined");
              return false;
            }
            if (vel < m_velocity_limit.at(iAx))
              m_velocity_limit.at(iAx) = vel;
          }

          if (has_acceleration_limits)
          {
            double acc;
            if (!Controller<T>::getControllerNh().getParam("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_acceleration", acc))
            {
              ROS_ERROR_STREAM("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_acceleration is not defined");
              return false;
            }
            m_acceleration_limit.at(iAx) = acc;
          }
          else
            m_acceleration_limit.at(iAx) = 10 * m_velocity_limit.at(iAx);


        Controller<T>::m_hw->getHandle(m_joint_names.at(iAx));
      }
      catch (...)
      {
        CNR_RETURN_FALSE(*Controller<T>::m_logger,
          "Controller '" + Controller<T>::getControllerNamespace() + "' failed in init. " + std::string("")
          + "The controlled joint named '" + m_joint_names.at(iAx) + "' is not managed by hardware_interface");
      }
    }

    CNR_RETURN_BOOL(*Controller<T>::m_logger, Controller<T>::dump_state());
  }

  const std::string& getJointName() const
  {
    return m_joint_names;
  }
protected:

  urdf::ModelInterfaceSharedPtr m_model;
  std::vector<std::string> m_joint_names;
  size_t                   m_nAx;

  std::vector<double>      m_upper_limit;
  std::vector<double>      m_lower_limit;
  std::vector<double>      m_velocity_limit;
  std::vector<double>      m_acceleration_limit;

};

} // cnr_controller_interface
#endif
