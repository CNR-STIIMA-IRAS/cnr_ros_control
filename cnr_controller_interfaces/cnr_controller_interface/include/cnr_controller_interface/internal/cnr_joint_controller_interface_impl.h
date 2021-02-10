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
  CNR_TRACE_START(this->m_logger);
  stopUpdateTransformationsThread();
  CNR_TRACE(this->m_logger, "OK");
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
  stopUpdateTransformationsThread();
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
  size_t l = __LINE__;
  try
  {
    m_cfrmt = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
    CNR_TRACE_START(this->m_logger);
    if(!Controller<T>::enterInit())
    {
      CNR_RETURN_FALSE(this->m_logger);
    }

    //=======================================
    // JOINT NAMES (RESOURCES)
    //=======================================
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
    //=======================================



    //=======================================
    // CHAIN
    //=======================================
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

    std::string robot_description_param;
    if(!this->getControllerNh().getParam("robot_description_param", robot_description_param ) )
    {
      if(!this->getRootNh().getParam("robot_description_param", robot_description_param ) )
      {
        CNR_RETURN_FALSE(this->m_logger, "'Neither '" + this->getControllerNamespace()
                          + "/robot_description_param' " + "nor '" + this->getRootNamespace()
                            + "/robot_description_param' are not in rosparam server.");
      }
    }

    std::string urdf_string;
    if(!ros::param::get(robot_description_param, urdf_string))
    {
      CNR_ERROR(this->m_logger, "\nWeird error in getting the parameter '" << robot_description_param
                  << "'. It was already checked the existence.\n");
      CNR_RETURN_FALSE(this->m_logger);
    }

    m_urdf_model = urdf::parseURDF(urdf_string);
    if(m_urdf_model == nullptr )
    {
      CNR_ERROR(this->m_logger, "Failing in parsing the URDF model from parameter '" + urdf_string + "'");
      CNR_RETURN_FALSE(this->m_logger);
    }

    rosdyn::LinkPtr root_link(new rosdyn::Link());  //link primitivo da cui parte la catena cinematica (world ad esempio)
    root_link->fromUrdf(m_urdf_model->root_link_);

    std::string error;
    if(!m_chain.init(error,root_link,base_link, tool_link))
    {
      CNR_ERROR(this->m_logger, "Failing in creating the Chain from the URDF model:\n\t" + error + "");
      CNR_RETURN_FALSE(this->m_logger);
    }
    //=======================================


    //=======================================
    // SELECT ACTIVE JOINT FOR THE CHAIN
    //=======================================
    m_chain.setInputJointsName(joint_names);
    //=======================================


    //=======================================
    // KINEMATICS LIMITS
    //=======================================
    std::string robot_description_planning_param;
    if(!this->getControllerNh().getParam("robot_description_planning_param", robot_description_planning_param ) )
    {
      if(!this->getRootNh().getParam("robot_description_planning_param", robot_description_planning_param ) )
      {
        CNR_ERROR(this->m_logger, "'Neither '" + this->getControllerNamespace()
                  + "/robot_description_planning_param' " + "nor '" + this->getRootNamespace()
                    + "/robot_description_param' are not in rosparam server.");
        CNR_RETURN_FALSE(this->m_logger);
      }
    }
    if(!ros::param::has(robot_description_planning_param))
    {
      CNR_ERROR(this->m_logger, "The parameter '" << robot_description_planning_param <<
                " does not exist (check the value of parameter '" << robot_description_planning_param <<"'");
      CNR_RETURN_FALSE(this->m_logger);
    }

    int res = m_chain.enforceLimitsFromRobotDescriptionParam(robot_description_planning_param, error);
    if(res==-1)
    {
      CNR_ERROR(this->m_logger, "Failing in setting the kin limits. Error: '" + error + "'");
      CNR_RETURN_FALSE(this->m_logger);
    }
    else if(res==-1)
    {
      CNR_WARN(this->m_logger, "Warning in setting the kin limits.: '" + error + "'");
    }


l = __LINE__;
    m_rstate.init(m_chain);
l = __LINE__;
    for (unsigned int iAx=0; iAx<m_chain.getActiveJointsNumber(); iAx++)
    {
      try
      {
        if(!Controller<T>::m_hw)
        {
          throw std::runtime_error("The HW is malformed!");
        }
        m_handler.handles_[m_chain.getJointName(iAx)] = Controller<T>::m_hw->getHandle(m_chain.getJointName(iAx));
      }
      catch (std::exception& e)
      {
        CNR_RETURN_FALSE(this->m_logger,
          "Controller '" + Controller<T>::getControllerNamespace() + "' failed in init. " + std::string("")
          + "The controlled joint named '" + m_chain.getJointName(iAx) + "' is not managed by hardware_interface.\
Error: " + std::string(e.what()));
      }
      catch (...)
      {
        CNR_RETURN_FALSE(this->m_logger,
          "Controller '" + Controller<T>::getControllerNamespace() + "' failed in init. " + std::string("")
          + "The controlled joint named '" + m_chain.getJointName(iAx) + "' is not managed by hardware_interface");
      }
      CNR_DEBUG(this->m_logger,
        "Controller '" + Controller<T>::getControllerNamespace() + std::string("'")
        + "The controlled joint named '" + m_chain.getJointName(iAx) + "' is managed by hardware_interface");
    }
  }
  catch(std::exception& e)
  {
    std::cerr << cnr_logger::RED() << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " ;
    std::cerr << "Exception at line: "
              << std::to_string(l) << " error: " + std::string(e.what())
              << std::endl;
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
  m_handler.flush(m_rstate, m_chain);
  CNR_RETURN_TRUE(this->m_logger);
}

template<int N,int MaxN,class H,class T>
bool JointController<N,MaxN,H,T>::exitStarting()
{
  CNR_TRACE_START(this->m_logger);

  int ffwd = rosdyn::ChainState<N,MaxN>::SECOND_ORDER | rosdyn::ChainState<N,MaxN>::FFWD_STATIC;

  startUpdateTransformationsThread(ffwd, 1.0/this->m_sampling_period);

  if(!Controller<T>::exitStarting())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

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

  m_handler.flush(m_rstate, m_chain);
  // NOTE: the transformations may take time, especially due the pseudo inversion of the Jacobian, to estimate the external wrench.
  // Therefore, they are executed in parallel
  //m_rstate.updateTransformations();

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->m_logger);
}

template<int N,int MaxN,class H,class T>
inline void JointController<N,MaxN,H,T>::startUpdateTransformationsThread(int ffwd_kin_type, double hz)
{
  stop_update_transformations_ = false;
  update_transformations_ = std::thread(
        &JointController<N,MaxN,H,T>::updateTransformationsThread, this, ffwd_kin_type, hz);
}

template<int N,int MaxN,class H,class T>
inline void JointController<N,MaxN,H,T>::stopUpdateTransformationsThread()
{
  stop_update_transformations_ = true;
  if(update_transformations_.joinable())
    update_transformations_.join();
}

template<int N,int MaxN,class H,class T>
inline void JointController<N,MaxN,H,T>::updateTransformationsThread(int ffwd_kin_type, double hz)
{
  mtx_.lock();
  rosdyn::ChainState<N,MaxN> rstate;
  mtx_.unlock();

  ros::Rate rt(hz);
  while(!stop_update_transformations_)
  {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      rstate.copy(m_rstate, m_rstate.ONLY_JOINT);
    }
    rstate.updateTransformations(m_chain, ffwd_kin_type);
    {
      std::lock_guard<std::mutex> lock(mtx_);
      m_rstate.copy(rstate, m_rstate.ONLY_CART);
    }
    rt.sleep();
  }
}

}  // cnr_controller_interface
}  // namespace cnr
#endif
