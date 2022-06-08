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

#ifndef CNR_CONTOLLER_INTERFACE__CNR_MULTI_CHAIN_CONTROLLER_INTERFACE_IMPL_H
#define CNR_CONTOLLER_INTERFACE__CNR_MULTI_CHAIN_CONTROLLER_INTERFACE_IMPL_H

#include <algorithm>
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
MultiChainController<H,T>::~MultiChainController()
{
  CNR_TRACE_START(this->m_logger);
  stopUpdateTransformationsThread();
  CNR_TRACE(this->m_logger, "OK");
}

template<class H,class T>
bool MultiChainController<H,T>::doInit()
{
  m_fkin_update_period = -1.0;
  return cnr::control::Controller<T>::doInit();
}
template<class H,class T>
bool MultiChainController<H,T>::doStarting(const ros::Time& time)
{
  return cnr::control::Controller<T>::doStarting(time);
}

template<class H,class T>
bool MultiChainController<H,T>::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  return cnr::control::Controller<T>::doUpdate(time,period);
}

template<class H,class T>
bool MultiChainController<H,T>::doStopping(const ros::Time& time)
{
  stopUpdateTransformationsThread();
  return cnr::control::Controller<T>::doStopping(time);
}

template<class H,class T>
bool MultiChainController<H,T>::doWaiting(const ros::Time& time)
{
  return cnr::control::Controller<T>::doWaiting(time);
}

template<class H,class T>
bool MultiChainController<H,T>::doAborting(const ros::Time& time)
{
  return cnr::control::Controller<T>::doAborting(time);
}

template<class H,class T>
bool MultiChainController<H,T>::enterInit()
{
  std::string what;
  size_t l = __LINE__;
  try
  {
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
    // CHAINS
    //=======================================
    std::vector<std::string> chain_ids;
    if(!ru::get(this->getControllerNamespace() + "/chain_ids", chain_ids, what ) )
    {
      if(!ru::get(this->getRootNamespace() + "/chain_ids", chain_ids, what ) )
      {
        CNR_RETURN_FALSE(this->m_logger, "'Neither '" + this->getControllerNamespace() + "/chain_ids' " +
                    "nor '"      + this->getRootNamespace() + "/chain_ids' are not in rosparam server.");
      }
    }
    //TODO FINd duplacete in the chain
    


    std::vector<std::string> base_links;
    if(!ru::get(this->getControllerNamespace() + "/base_links", base_links, what ) )
    {
      if(!ru::get(this->getRootNamespace() + "/base_links", base_links, what ) )
      {
        CNR_RETURN_FALSE(this->m_logger, "'Neither '" + this->getControllerNamespace() + "/base_links' " +
                    "nor '"      + this->getRootNamespace() + "/base_links' are not in rosparam server.");
      }
    }
    if(base_links.size()!=chain_ids.size())
    {
      CNR_RETURN_FALSE(this->m_logger, "'Mismatch in dimensionf of 'base_links' and 'chain_ids'.");
    }

    std::vector<std::string> tool_links;
    if(!ru::get(this->getControllerNamespace() + "/tool_links", tool_links, what ) )
    {
      if(!ru::get(this->getRootNamespace() + "/tool_links", tool_links, what ) )
      {
        CNR_RETURN_FALSE(this->m_logger, "'Neither '" + this->getControllerNamespace() + "/tool_links' " +
                  "nor '"      + this->getRootNamespace() + "/tool_links' are not in rosparam server.");
      }
    }
    if(tool_links.size()!=chain_ids.size())
    {
      CNR_RETURN_FALSE(this->m_logger, "'Mismatch in dimensionf of 'tool_links' and 'chain_ids'.");
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
    for(size_t i=0;i<chain_ids.size();i++)
    {
      if(m_chain.find(chain_ids.at(i)!=m_chain.end()))
      {
        CNR_ERROR(this->m_logger, "Multiple Chain with same ID!");
        CNR_RETURN_FALSE(this->m_logger);
      }
      if(!m_chain[chain_ids.at(i)].init(error,m_root_link,base_links.at(i), tool_link.at(i)))
      {
        CNR_ERROR(this->m_logger, "Failing in creating the Chain from the URDF model:\n\t" + error + "");
        CNR_RETURN_FALSE(this->m_logger);
      }
    }
    //=======================================


    //=======================================
    // SELECT ACTIVE JOINT FOR THE CHAIN
    //=======================================
    CNR_DEBUG(this->m_logger, "Controlled joint names: " << cnr::control::to_string(joint_names));
    std::map<std::string, std::vector<std::string> > chain_joint_names;
    std::vector<std::vector<std::string>::iterator> joint_names_used;
    for(auto it = m_chain.begin(); it != m_chain.end(); it++)
    {
      auto & chain = it->second;
      auto ajns = chain.getActiveJointsName();
      CNR_DEBUG(this->m_logger, "Default Chain Active Joint Names: " << cnr::control::to_string(ajns));
      
      chain_joint_names[it->first] = std::vector<std::string>{};
      for(auto const & ajn : ajns)
      {
        auto jt = std::find(joint_names.begin(), joint_names.end());
        if( jt == joint_names.end())
        {
          continue;
        }

        if( std::find(joint_names_used.begin(), joint_names_used.end(), jt ) )
        {
          CNR_ERROR(this->m_logger, "The joint '"+ *jt "' is an active joint in multiple chain");
          CNR_RETURN_FALSE(this->m_logger);
        } 

        joint_names_used.push_back(jt);
        chain_joint_names[it->first].push_back(ajn);
      }
    
      int ok_coherence_jnames_and_chain = chain.at(id).setInputJointsName(chain_joint_names);
      if(ok_coherence_jnames_and_chain!=1)
      {
        CNR_ERROR(this->m_logger, "Mismatch between the chain names and the controlled joint names. Abort.");
        CNR_RETURN_FALSE(this->m_logger);
      }
      if(chain.getActiveJointsNumber()!=joint_names.size())
      {
        CNR_ERROR(this->m_logger, "Mismatch of the dimension of the chain names and the controlled joint names. Abort.");
        CNR_RETURN_FALSE(this->m_logger);
      }
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

l = __LINE__;
    for(auto & c : m_chain )
    {
      auto & id = c.first;
      auto & chain = c.second;
    
      int res = chain.enforceLimitsFromRobotDescriptionParam(robot_description_planning_param, error);
      if(res==-1)
      {
        CNR_ERROR(this->m_logger, "Failing in setting the kin limits. Error: '" + error + "'");
        CNR_RETURN_FALSE(this->m_logger);
      }
      else if(res==0)
      {
        CNR_WARN(this->m_logger, "Warning in setting the kin limits.: '" + error + "'");
      }
    
      m_rstate[id].init(chain);
       
      for(unsigned int iAx=0; iAx<chain.getActiveJointsNumber(); iAx++)
      {
        try
        {
          if(!Controller<T>::m_hw)
          {
            throw std::runtime_error("The HW is malformed!");
          }
          if(m_handlers.find(id)!=m_handlers.end())
          {
            CNR_ERROR(this->m_logger, " Error: ultiple handlers with same id");
            CNR_RETURN_FALSE(this->m_logger); 
          }
          m_handlers[id].handles_[chain.getActiveJointName(iAx)] =
                Controller<T>::m_hw->getHandle(chain.getActiveJointName(iAx));
        }
        catch(std::exception& e)
        {
          CNR_RETURN_FALSE(this->m_logger,
            "Controller '" + Controller<T>::getControllerNamespace() + "' failed in init. " + std::string("")
            + "The controlled joint named '"+chain.getActiveJointName(iAx)+"' is not managed by hardware_interface."
                "Error: " + std::string(e.what()));
        }
        catch(...)
        {
          CNR_RETURN_FALSE(this->m_logger,
            "Controller '" + Controller<T>::getControllerNamespace() + "' failed in init. " + std::string("")
            + "The controlled joint named '"+chain.getActiveJointName(iAx)+"' is not managed by hardware_interface");
        }
        CNR_DEBUG(this->m_logger,
          "Controller '" + Controller<T>::getControllerNamespace() + std::string("'")
          + "The controlled joint named '" + chain.getActiveJointName(iAx) + "' is managed by hardware_interface");
      }
      CNR_DEBUG(this->m_logger, "Q sup  : " << eigen_utils::to_string(chain.getQMax()  ));
      CNR_DEBUG(this->m_logger, "Q inf  : " << eigen_utils::to_string(chain.getQMin()  ));
      CNR_DEBUG(this->m_logger, "Qd max : " << eigen_utils::to_string(chain.getDQMax() ));
      CNR_DEBUG(this->m_logger, "Qdd max: " << eigen_utils::to_string(chain.getDDQMax()));
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

template<class H,class T>
bool MultiChainController<H,T>::enterStarting()
{
  CNR_TRACE_START(this->m_logger);
  if(!Controller<T>::enterStarting())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  for(auto & c : m_chain)
  {
    auto & id = c.first;
    auto & chain = c.second;
    CNR_DEBUG(this->m_logger, "HW Status\n" << std::to_string(m_handlers.at(id)) );
    CNR_DEBUG(this->m_logger, "Chain: " << std::to_string(chain.getActiveJointsNumber()) );
    CNR_DEBUG(this->m_logger, "First joint name: " << chain.getActiveJointsName().front() );
    CNR_DEBUG(this->m_logger, "Last joint name: " << chain.getActiveJointsName().back() );
    m_handlers.at(chain.first).flush(m_rstate.at(id), chain);

    CNR_DEBUG(this->m_logger, "Position: " << eigen_utils::to_string(m_rstate.at(id).q()) );
    CNR_DEBUG(this->m_logger, "Velocity: " << eigen_utils::to_string(m_rstate.at(id).qd()) );
    CNR_DEBUG(this->m_logger, "Effort  : " << eigen_utils::to_string(m_rstate.at(id).effort()) );
  }
  CNR_RETURN_TRUE(this->m_logger);
}

template<class H,class T>
bool MultiChainController<H,T>::exitStarting()
{
  CNR_TRACE_START(this->m_logger);

  int ffwd = rosdyn::ChainState::SECOND_ORDER | rosdyn::ChainState::FFWD_STATIC;

  if(m_fkin_update_period>0)
    startUpdateTransformationsThread(ffwd, 1.0/this->m_fkin_update_period);

  if(!Controller<T>::exitStarting())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  CNR_RETURN_TRUE(this->m_logger);
}

template<class H,class T>
bool MultiChainController<H,T>::enterUpdate()
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->m_logger);
  if(!Controller<T>::enterUpdate())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  for(auto & c : m_chain)
  {
    auto & id = c.first;
    auto & chain = c.second;
    m_handlers.at(id).flush(m_rstate.at(id), chain);
    // NOTE: the transformations may take time, especially due the pseudo inversion of the Jacobian, to estimate the external wrench.
    // Therefore, they are executed in parallel
  }

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->m_logger);
}

template<class H,class T>
inline bool MultiChainController<H,T>::startUpdateTransformationsThread(int ffwd_kin_type, double hz)
{
  CNR_TRACE_START(this->m_logger);
  stop_update_transformations_ = false;
  update_transformations_runnig_ = false;
  CNR_DEBUG(this->logger(), "Creating the fkin update thread");
  update_transformations_ = std::thread(
        &MultiChainController<H,T>::updateTransformationsThread, this, ffwd_kin_type, hz);
  CNR_DEBUG(this->logger(), "Fkin Thread created");
  
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
inline void MultiChainController<H,T>::stopUpdateTransformationsThread()
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
inline void MultiChainController<H,T>::updateTransformationsThread(int ffwd_kin_type, double hz)
{
  CNR_TRACE_START(this->m_logger);
  rosdyn::ChainState rstate;

  {
    std::lock_guard<std::mutex> lock(this->mtx_);
    if(!rstate.init(m_chain))
    {
       CNR_FATAL(this->m_logger, "Chain failure!");
       CNR_RETURN_NOTOK(this->m_logger, void());
    }
  }

  ros::Rate rt(hz);
  while(!stop_update_transformations_)
  {
    for(auto & rs : m_rstate)
    {
      auto & id = rs.first;
      auto & rstate = rs.second;
      rstate.copy(rstate, rstate.ONLY_JOINT);
      rstate.updateTransformations(m_chain.at(id), ffwd_kin_type);
      rstate.copy(rstate, rstate.ONLY_CART);

      if(!this->update_transformations_runnig_)
      {
        CNR_INFO(this->logger(), "First state update ;)");
        this->update_transformations_runnig_ = true;
      }
    }
    rt.sleep();
  }
  CNR_RETURN_OK(this->m_logger, void());
}

template<class H,class T>
inline const rosdyn::Chain& MultiChainController<H,T>::chain(const std::string& id) const
{
  return m_chain.at(id);
}

template<class H,class T>
inline rosdyn::Chain& MultiChainController<H,T>::chainNonConst(const std::string& id)
{
  return m_chain.at(id);
}

template<class H,class T>
inline const rosdyn::ChainState& MultiChainController<H,T>::chainState(const std::string& id) const
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id);
}

template<class H,class T>
inline rosdyn::ChainState& MultiChainController<H,T>::chainState(const std::string& id)
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id);
}

template<class H,class T>
inline const rosdyn::VectorXd& MultiChainController<H,T>::getPosition(const std::string& id) const
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id).q();
}

template<class H,class T>
inline const rosdyn::VectorXd& MultiChainController<H,T>::getVelocity(const std::string& id) const
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id).qd();
}

template<class H,class T>
inline const rosdyn::VectorXd& MultiChainController<H,T>::getAcceleration(const std::string& id) const
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id).qdd();
}

template<class H,class T>
inline const rosdyn::VectorXd& MultiChainController<H,T>::getEffort(const std::string& id) const
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id).effort();
}

template<class H,class T>
inline double MultiChainController<H,T>::getPosition(const std::string& id,int idx) const
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id).q(idx);
}

template<class H,class T>
inline double MultiChainController<H,T>::getVelocity(const std::string& id,int idx) const
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id).qd(idx);
}

template<class H,class T>
inline double MultiChainController<H,T>::getAcceleration(const std::string& id,int idx) const
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id).qdd(idx);
}

template<class H,class T>
inline double MultiChainController<H,T>::getEffort(const std::string& id,int idx) const
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id).effort(idx);
}

template<class H,class T>
inline const Eigen::Affine3d& MultiChainController<H,T>::getToolPose(const std::string& id) const
{
  if(m_fkin_update_period<=0)
    throw std::runtime_error("The 'kin_update_period' has not been set, and therefore the fkin is not computed!");
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id).toolPose();
}

template<class H,class T>
inline const Eigen::Vector6d& MultiChainController<H,T>::getTwist(const std::string& id) const
{
  if(m_fkin_update_period<=0)
    throw std::runtime_error("The 'kin_update_period' has not been set, and therefore the fkin is not computed!");
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id).toolTwist();
}

template<class H,class T>
inline const Eigen::Vector6d& MultiChainController<H,T>::getTwistd(const std::string& id) const
{
  if(m_fkin_update_period<=0)
    throw std::runtime_error("The 'kin_update_period' has not been set, and therefore the fkin is not computed!");
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id).toolTwistd();
}

template<class H,class T>
inline const rosdyn::Matrix6Xd& MultiChainController<H,T>::getJacobian(const std::string& id) const
{
  if(m_fkin_update_period<=0)
    throw std::runtime_error("The 'kin_update_period' has not been set, and therefore the fkin is not computed!");
  std::lock_guard<std::mutex> lock(this->mtx_);
  return m_rstate.at(id).toolJacobian();
}



}  // cnr_controller_interface
}  // namespace cnr
#endif  // CNR_CONTOLLER_INTERFACE__CNR_MULTI_CHAIN_CONTROLLER_INTERFACE_IMPL_H
