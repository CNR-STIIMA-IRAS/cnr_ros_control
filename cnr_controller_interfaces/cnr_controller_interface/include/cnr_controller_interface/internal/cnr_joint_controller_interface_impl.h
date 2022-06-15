/*
 *  Software License Agreement(New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy(CNR)
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once // workaround clang-tidy qtcreator

#ifndef CNR_CONTOLLER_INTERFACE__CNR_JOINT_CONTROLLER_INTERFACE_IMPL_H
#define CNR_CONTOLLER_INTERFACE__CNR_JOINT_CONTROLLER_INTERFACE_IMPL_H

#include <sstream>
#include <ros/ros.h>

#include <rosparam_utilities/rosparam_utilities.h>
#include <cnr_logger/cnr_logger.h>
#include <rosdyn_chain_state/chain_state.h>
#include <cnr_controller_interface/internal/cnr_handles.h>
#include <cnr_controller_interface/cnr_joint_controller_interface.h>
#include <rosdyn_core/primitives.h>
#include <rosdyn_chain_state/chain_state.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace ru = rosparam_utilities;

namespace cnr
{
namespace control
{

template<class H,class T>
JointController<H,T>::~JointController()
{
  CNR_TRACE_START(this->m_logger);
  stopUpdateTransformationsThread();
  CNR_TRACE(this->m_logger, "OK");
}

template<class H,class T>
bool JointController<H,T>::doInit()
{
  m_fkin_update_period = -1.0;
  return cnr::control::Controller<T>::doInit();
}
template<class H,class T>
bool JointController<H,T>::doStarting(const ros::Time& time)
{
  return cnr::control::Controller<T>::doStarting(time);
}

template<class H,class T>
bool JointController<H,T>::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  return cnr::control::Controller<T>::doUpdate(time,period);
}

template<class H,class T>
bool JointController<H,T>::doStopping(const ros::Time& time)
{
  stopUpdateTransformationsThread();
  return cnr::control::Controller<T>::doStopping(time);
}

template<class H,class T>
bool JointController<H,T>::doWaiting(const ros::Time& time)
{
  return cnr::control::Controller<T>::doWaiting(time);
}

template<class H,class T>
bool JointController<H,T>::doAborting(const ros::Time& time)
{
  return cnr::control::Controller<T>::doAborting(time);
}

template<class H,class T>
bool JointController<H,T>::enterInit()
{
  std::string what;
  size_t l = __LINE__;
  
  
  try
  {
    //====================================================
    std::lock_guard<std::mutex> lock(this->m_chain_mtx);
    //====================================================

    m_cfrmt = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
    CNR_TRACE_START(this->m_logger);
    if(!Controller<T>::enterInit())
    {
      CNR_RETURN_FALSE(this->m_logger);
    }

    m_fkin_update_period = -1;
    if(!ru::get(this->getControllerNamespace() + "/kin_update_period", m_fkin_update_period, what, &m_fkin_update_period))
    {
      CNR_WARN(this->m_logger, what);
      CNR_WARN(this->m_logger, "The chain status will not be updated.");;
    }

    //=======================================
    // JOINT NAMES(RESOURCES)
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
    if(!ru::get(this->getControllerNamespace() + "/base_link", base_link, what ) )
    {
      if(!ru::get(this->getRootNamespace() + "/base_link", base_link, what ) )
      {
        CNR_RETURN_FALSE(this->m_logger, "'Neither '" + this->getControllerNamespace() + "/base_link' " +
                    "nor '"      + this->getRootNamespace() + "/base_link' are not in rosparam server.");
      }
    }

    std::string tool_link;
    if(!ru::get(this->getControllerNamespace() + "/tool_link", tool_link, what ) )
    {
      if(!ru::get(this->getRootNamespace() + "/tool_link", tool_link, what ) )
      {
        CNR_RETURN_FALSE(this->m_logger, "'Neither '" + this->getControllerNamespace() + "/tool_link' " +
                  "nor '"      + this->getRootNamespace() + "/tool_link' are not in rosparam server.");
      }
    }

    std::string robot_description_param;
    if(!ru::get(this->getControllerNamespace() + "/robot_description_param", robot_description_param, what ) )
    {
      if(!ru::get(this->getRootNamespace() + "/robot_description_param", robot_description_param, what ) )
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

    NEW_HEAP(m_root_link, rosdyn::Link() );
    m_root_link->fromUrdf(GET(m_urdf_model->root_link_));
    std::string error;
    if(!m_chain.init(error,m_root_link,base_link, tool_link)
    || !m_chain_threaded.init(error,m_root_link,base_link, tool_link))
    {
      CNR_ERROR(this->m_logger, "Failing in creating the Chain from the URDF model:\n\t" + error + "");
      CNR_RETURN_FALSE(this->m_logger);
    }

    //=======================================


    //=======================================
    // SELECT ACTIVE JOINT FOR THE CHAIN
    //=======================================
    CNR_DEBUG(this->m_logger, "Controlled joint names: " << cnr::control::to_string(joint_names));
    CNR_DEBUG(this->m_logger, "Default Chain Active Joint Names: " << cnr::control::to_string(this->m_chain.getActiveJointsName()));

    int ok_coherence_jnames_and_chain = m_chain.setInputJointsName(joint_names);
    if(ok_coherence_jnames_and_chain!=1)
    {
      CNR_ERROR(this->m_logger, "Mismatch between the chain names and the controlled joint names. Abort.");
      CNR_RETURN_FALSE(this->m_logger);
    }
    m_chain_threaded.setInputJointsName(joint_names);

    if(m_chain.getActiveJointsNumber()!=joint_names.size())
    {
      CNR_ERROR(this->m_logger, "Mismatch of the dimension of the chain names and the controlled joint names. Abort.");
      CNR_RETURN_FALSE(this->m_logger);
    }
    //=======================================


    //=======================================
    // KINEMATICS LIMITS
    //=======================================
    std::string robot_description_planning_param;
    if(!ru::get(this->getControllerNamespace() + "/robot_description_planning_param", robot_description_planning_param, what ) )
    {
      if(!ru::get(this->getRootNamespace() + "/robot_description_planning_param", robot_description_planning_param, what ) )
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
                " does not exist(check the value of parameter '" << robot_description_planning_param <<"'");
      CNR_RETURN_FALSE(this->m_logger);
    }

    int res = m_chain.enforceLimitsFromRobotDescriptionParam(robot_description_planning_param, error);
    if(res==-1)
    {
      CNR_ERROR(this->m_logger, "Failing in setting the kin limits. Error: '" + error + "'");
      CNR_RETURN_FALSE(this->m_logger);
    }
    else if(res==0)
    {
      CNR_WARN(this->m_logger, "Warning in setting the kin limits.: '" + error + "'");
    }
    m_chain_threaded.enforceLimitsFromRobotDescriptionParam(robot_description_planning_param, error);

l = __LINE__;
    {
      CNR_DEBUG(this->m_logger, "lock rstate mtx");
      std::lock_guard<std::mutex> lock(this->m_rstate_mtx);
      m_rstate.init(m_chain);
      m_rstate_threaded.init(m_chain);
    }
l = __LINE__;
    for(unsigned int iAx=0; iAx<m_chain.getActiveJointsNumber(); iAx++)
    {
      try
      {
        if(!Controller<T>::m_hw)
        {
          throw std::runtime_error("The HW is malformed!");
        }
        m_handler.handles_[m_chain.getActiveJointName(iAx)] =
              Controller<T>::m_hw->getHandle(m_chain.getActiveJointName(iAx));
      }
      catch(std::exception& e)
      {
        CNR_RETURN_FALSE(this->m_logger,
          "Controller '" + Controller<T>::getControllerNamespace() + "' failed in init. " + std::string("")
          + "The controlled joint named '"+m_chain.getActiveJointName(iAx)+"' is not managed by hardware_interface."
              "Error: " + std::string(e.what()));
      }
      catch(...)
      {
        CNR_RETURN_FALSE(this->m_logger,
          "Controller '" + Controller<T>::getControllerNamespace() + "' failed in init. " + std::string("")
          + "The controlled joint named '"+m_chain.getActiveJointName(iAx)+"' is not managed by hardware_interface");
      }
      CNR_DEBUG(this->m_logger,
        "Controller '" + Controller<T>::getControllerNamespace() + std::string("'")
        + "The controlled joint named '" + m_chain.getActiveJointName(iAx) + "' is managed by hardware_interface");
    }
    m_handler.init(m_chain);

    CNR_DEBUG(this->m_logger, "Q sup  : " << eigen_utils::to_string(m_chain.getQMax()  ));
    CNR_DEBUG(this->m_logger, "Q inf  : " << eigen_utils::to_string(m_chain.getQMin()  ));
    CNR_DEBUG(this->m_logger, "Qd max : " << eigen_utils::to_string(m_chain.getDQMax() ));
    CNR_DEBUG(this->m_logger, "Qdd max: " << eigen_utils::to_string(m_chain.getDDQMax()));
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

template<class H,class T>
bool JointController<H,T>::enterStarting()
{
  CNR_TRACE_START(this->m_logger);
  if(!Controller<T>::enterStarting())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  {  
    CNR_DEBUG(this->m_logger, "HW Status\n" << std::to_string(m_handler) );
    CNR_DEBUG(this->m_logger, "Chain: " << std::to_string(m_chain.getActiveJointsNumber()) );
    CNR_DEBUG(this->m_logger, "First joint name: " << m_chain.getActiveJointsName().front() );
    CNR_DEBUG(this->m_logger, "Last joint name: " << m_chain.getActiveJointsName().back() );
    {
      CNR_DEBUG(this->m_logger, "lock rstate mtx");
      std::lock_guard<std::mutex> lock(this->m_rstate_mtx);
      m_handler.flush(m_rstate);
      CNR_DEBUG(this->m_logger, "Position: " << eigen_utils::to_string(m_rstate.q()) );
      CNR_DEBUG(this->m_logger, "Velocity: " << eigen_utils::to_string(m_rstate.qd()) );
      CNR_DEBUG(this->m_logger, "Effort  : " << eigen_utils::to_string(m_rstate.effort()) );
    }
  }

  CNR_RETURN_TRUE(this->m_logger);
}

template<class H,class T>
bool JointController<H,T>::exitStarting()
{
  CNR_TRACE_START(this->m_logger);

  int ffwd = rosdyn::ChainState::SECOND_ORDER | rosdyn::ChainState::FFWD_STATIC;

  if(m_fkin_update_period>0)
  {
    stop_update_transformations_ = false;
    update_transformations_runnig_ = false;
    CNR_DEBUG(this->m_logger, "Creating the fkin update thread");

    update_transformations_ = std::thread(
        &JointController<H,T>::updateTransformationsThread, this, ffwd, 1.0/this->m_fkin_update_period);

    CNR_DEBUG(this->m_logger, "Fkin Thread created");

    double timeout = 1.0;
    ros::Time st = ros::Time::now();
    while(!update_transformations_runnig_)
    {
      ros::Duration(0.005).sleep();
      if((ros::Time::now()-st).toSec()>timeout)
      {
        CNR_ERROR(this->m_logger,"The thread that updates the fkin didn't start within the timeout of "<< timeout << ". Abort");
        CNR_RETURN_TRUE(this->m_logger);
      }
    }
  }
  //startUpdateTransformationsThread(ffwd, 1.0/this->m_fkin_update_period);

  if(!Controller<T>::exitStarting())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  CNR_RETURN_TRUE(this->m_logger);
}

template<class H,class T>
bool JointController<H,T>::enterUpdate()
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->m_logger);
  if(!Controller<T>::enterUpdate())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  {
    CNR_DEBUG(this->m_logger, "lock rstate mtx");
    std::lock_guard<std::mutex> lock(this->m_rstate_mtx);
    m_handler.flush(m_rstate);
  }
  
  // NOTE: the transformations may take time, especially due the pseudo inversion of the Jacobian, to estimate the external wrench.
  // Therefore, they are executed in parallel
  //m_rstate.updateTransformations();

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->m_logger);
}

template<class H,class T>
inline bool JointController<H,T>::startUpdateTransformationsThread(int ffwd_kin_type, double hz)
{
  CNR_TRACE_START(this->m_logger);
  stop_update_transformations_ = false;
  update_transformations_runnig_ = false;
  CNR_DEBUG(this->m_logger, "Creating the fkin update thread");

  update_transformations_ = std::thread(
        &JointController<H,T>::updateTransformationsThread, this, ffwd_kin_type, hz);

  CNR_DEBUG(this->m_logger, "Fkin Thread created");

  double timeout = 1.0;
  ros::Time st = ros::Time::now();
  while(!update_transformations_runnig_)
  {
    ros::Duration(0.005).sleep();
    if((ros::Time::now()-st).toSec()>timeout)
    {
      CNR_ERROR(this->m_logger,"The thread that updates the fkin didn't start within the timeout of "<< timeout << ". Abort");
      CNR_RETURN_TRUE(this->m_logger);  
    }
  }
  CNR_RETURN_TRUE(this->m_logger);
}

template<class H,class T>
inline void JointController<H,T>::stopUpdateTransformationsThread()
{
  CNR_TRACE_START(this->m_logger);
  if(m_fkin_update_period>0)
  {
    stop_update_transformations_ = true;
    if(update_transformations_.joinable())
      update_transformations_.join();
  }
  CNR_RETURN_OK(this->m_logger, void());
}

template<class H,class T>
inline void JointController<H,T>::updateTransformationsThread(int ffwd_kin_type, double hz)
{
  CNR_TRACE_START(this->m_logger);
  ros::Rate rt(hz);
  while(!this->stop_update_transformations_)
  {
    {
      CNR_DEBUG(this->m_logger, "lock rstate mtx");
      std::lock_guard<std::mutex> lock(this->m_rstate_mtx);
      m_rstate_threaded.copy(m_rstate, rosdyn::ChainState::ONLY_JOINT);
    }
    
    m_rstate_threaded.updateTransformations(m_chain_threaded, ffwd_kin_type);
    
    {
      CNR_DEBUG(this->m_logger, "lock rstate mtx");
      std::lock_guard<std::mutex> lock(this->m_rstate_mtx);
      m_rstate.copy(m_rstate_threaded, rosdyn::ChainState::ONLY_CART);
    }

    if(!this->update_transformations_runnig_)
    {
      CNR_INFO(this->m_logger, "First state update ;)");
      this->update_transformations_runnig_ = true;
    }
    rt.sleep();
  }
  CNR_RETURN_OK(this->m_logger, void());

}

template<class H,class T>
inline const rosdyn::Chain& JointController<H,T>::chain() const
{
  std::lock_guard<std::mutex> lock(this->m_chain_mtx);
  return m_chain;
}

template<class H,class T>
inline const unsigned int& JointController<H,T>::nAx( ) const 
{
  std::lock_guard<std::mutex> lock(this->m_chain_mtx);
  return m_chain.getActiveJointsNumber(); 
}

template<class H,class T>
inline const std::vector<std::string>& JointController<H,T>::jointNames( ) const
{
  std::lock_guard<std::mutex> lock(this->m_chain_mtx);
  return m_chain.getActiveJointsName(); 
}


template<class H,class T>
inline rosdyn::Chain& JointController<H,T>::chainNonConst()
{
  std::lock_guard<std::mutex> lock(this->m_chain_mtx);
  return m_chain;
}

template<class H,class T>
inline const rosdyn::ChainState& JointController<H,T>::chainState() const
{
  return m_rstate;
}

template<class H,class T>
inline rosdyn::ChainState& JointController<H,T>::chainState()
{
  return m_rstate;
}

template<class H,class T>
inline const rosdyn::VectorXd& JointController<H,T>::getPosition( ) const
{
  return m_rstate.q();
}

template<class H,class T>
inline const rosdyn::VectorXd& JointController<H,T>::getVelocity( ) const
{
  return m_rstate.qd();
}

template<class H,class T>
inline const rosdyn::VectorXd& JointController<H,T>::getAcceleration( ) const
{
  return m_rstate.qdd();
}

template<class H,class T>
inline const rosdyn::VectorXd& JointController<H,T>::getEffort( ) const
{
  return m_rstate.effort();
}

template<class H,class T>
inline double JointController<H,T>::getPosition(int idx) const
{
  return m_rstate.q(idx);
}

template<class H,class T>
inline double JointController<H,T>::getVelocity(int idx) const
{
  return m_rstate.qd(idx);
}

template<class H,class T>
inline double JointController<H,T>::getAcceleration(int idx) const
{
  return m_rstate.qdd(idx);
}

template<class H,class T>
inline double JointController<H,T>::getEffort(int idx) const
{
  return m_rstate.effort(idx);
}

template<class H,class T>
inline const Eigen::Affine3d& JointController<H,T>::getToolPose( ) const
{
  if(m_fkin_update_period<=0)
    throw std::runtime_error("The 'kin_update_period' has not been set, and therefore the fkin is not computed!");
  return m_rstate.toolPose();
}

template<class H,class T>
inline const Eigen::Vector6d& JointController<H,T>::getTwist( ) const
{
  if(m_fkin_update_period<=0)
    throw std::runtime_error("The 'kin_update_period' has not been set, and therefore the fkin is not computed!");
  return m_rstate.toolTwist();
}

template<class H,class T>
inline const Eigen::Vector6d& JointController<H,T>::getTwistd( ) const
{
  if(m_fkin_update_period<=0)
    throw std::runtime_error("The 'kin_update_period' has not been set, and therefore the fkin is not computed!");
  return m_rstate.toolTwistd();
}

template<class H,class T>
inline const rosdyn::Matrix6Xd& JointController<H,T>::getJacobian( ) const
{
  if(m_fkin_update_period<=0)
    throw std::runtime_error("The 'kin_update_period' has not been set, and therefore the fkin is not computed!");
  return m_rstate.toolJacobian();
}



}  // cnr_controller_interface
}  // namespace cnr
#endif
