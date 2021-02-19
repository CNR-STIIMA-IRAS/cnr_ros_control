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
#pragma once // workaround clang-tidy in qtcreator

#ifndef CNR_CONTOLLER_INTERFACE__CNR_JOINT_COMMAND_CONTROLLER_INTERFACE_IMPL_H
#define CNR_CONTOLLER_INTERFACE__CNR_JOINT_COMMAND_CONTROLLER_INTERFACE_IMPL_H

#include <std_msgs/Int64.h>
#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>
#include <rosdyn_core/kinematics_saturation.h>
#include <rosdyn_utilities/chain_state.h>
#include <cnr_controller_interface/internal/cnr_handles.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace cnr
{
namespace control
{


template<class H,class T>
inline JointCommandController<H,T>::~JointCommandController()
{
  CNR_TRACE_START(this->m_logger);
  this->stopUpdateTransformationsThread();
  CNR_TRACE(this->m_logger, "OK");
}

template<class H,class T>
inline bool JointCommandController<H,T>::doInit()
{
  return true;
}

template<class H,class T>
inline bool JointCommandController<H,T>::doStarting(const ros::Time& /*time*/)
{
  return true;
}

template<class H,class T>
inline bool JointCommandController<H,T>::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  return true;
}

template<class H,class T>
inline bool JointCommandController<H,T>::doStopping(const ros::Time& /*time*/)
{
  this->stopUpdateTransformationsThread();
  return true;
}

template<class H,class T>
inline bool JointCommandController<H,T>::doWaiting(const ros::Time& /*time*/)
{
  return true;
}

template<class H,class T>
inline bool JointCommandController<H,T>::doAborting(const ros::Time& /*time*/)
{
  return true;
}

template<class H,class T>
inline bool JointCommandController<H,T>::enterInit()
{
  CNR_TRACE_START(this->m_logger);
  if(!JointController<H,T>::enterInit())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  m_priority = QD_PRIORITY;
  m_target.init(this->m_chain);
  m_last_target.init(this->m_chain);

  this->template add_subscriber<std_msgs::Int64>("/speed_ovr" , 1,
                     boost::bind(&JointCommandController<H,T>::overrideCallback, this, _1), false);
  this->template add_subscriber<std_msgs::Int64>("/safe_ovr_1", 1,
                     boost::bind(&JointCommandController<H,T>::safeOverrideCallback_1, this, _1), false);
  this->template add_subscriber<std_msgs::Int64>("/safe_ovr_2", 1,
                 boost::bind(&JointCommandController<H,T>::safeOverrideCallback_2, this, _1), false);

  if(!(this->getControllerNh().getParam("max_velocity_multiplier", m_max_velocity_multiplier)) )
  {
    m_max_velocity_multiplier = 10;
  }

  m_override = 1;
  m_safe_override_1 = 1;
  m_safe_override_2 = 1;

  CNR_RETURN_TRUE(this->m_logger);
}

template<class H,class T>
inline bool JointCommandController<H,T>::enterStarting()
{
  CNR_TRACE_START(this->m_logger);
  if(!JointController<H,T>::enterStarting())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  m_target.setZero(this->m_chain);
  m_target.q() = this->getPosition();

  // in the exitStarting, the updateThread with the ffwd is launched

  CNR_RETURN_TRUE(this->m_logger);
}

template<class H,class T>
inline bool JointCommandController<H,T>::enterUpdate()
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->m_logger);
  if(!JointController<H,T>::enterUpdate())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }
  m_last_target.copy(m_target, m_target.FULL_STATE);
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->m_logger);
}

template<class H,class T>
inline bool JointCommandController<H,T>::exitUpdate()
#define SP std::fixed  << std::setprecision(5)
#define TP(X) eigen_utils::to_string(X)
{
  std::stringstream report;
  bool print_report = false;
  double throttle_time = 1.0;
  CNR_TRACE_START_THROTTLE_DEFAULT(this->m_logger);

  try
  {
    report << "==========\n";
    report << "Priorty            : " << std::to_string(m_priority) << "\n";
    report << "upper limit        : " << TP(this->m_chain.getQMax()) << "\n";
    report << "lower limit        : " << TP(this->m_chain.getQMin()) << "\n";
    report << "Speed Limit        : " << TP(this->m_chain.getDQMax()) << "\n";
    report << "Acceleration Limit : " << TP(this->m_chain.getDDQMax()) << "\n";
    report << "----------\n";
    // ============================== ==============================
    ROS_DEBUG_ONCE("Set Target according to Priority");
    auto nominal_qd = m_target.q();
    if(m_priority == Q_PRIORITY)
    {
      if(std::isnan(eigen_utils::norm(m_target.q())))
      {
        print_report = true;
        report << "SAFETY CHECK - Received a position with nan values... superimposed to zero!\n";
        m_target.q() = m_last_target.q();
      }
      nominal_qd =(m_target.q() - m_last_target.q()) / this->m_dt.toSec();
    }
    else if(m_priority == QD_PRIORITY)
    {
      nominal_qd = m_target.qd();
      if(std::isnan(eigen_utils::norm(nominal_qd)))
      {
        print_report = true;
        eigen_utils::setZero(nominal_qd);
      }
    }
    report << "Nominal command qd(input)  : " << TP(nominal_qd) << "\n";
    report << "----------\n";
    // ============================== ==============================


    // ============================== ==============================
    auto saturated_qd = nominal_qd;

    if(rosdyn::saturateSpeed(this->m_chain, saturated_qd, this->getVelocity(), this->getPosition(),
                               this->m_sampling_period, m_max_velocity_multiplier, true, &report))
    {
      print_report = true;
    }
    // ==============================
  }
  catch(...)
  {
    CNR_WARN(this->m_logger,"something wrong in JointTargetFilter::update");
    m_target.q()  = m_last_target.q();
    eigen_utils::setZero(m_target.qd());
  }

  report<< "q  trg: " << TP(m_target.q()) << "\n";
  report<< "qd trg: " << TP(m_target.qd()) << "\n";
  report<< "ef trg: " << TP(m_target.effort()) << "\n";

  this->m_handler.update(m_target, this->m_chain);

  for(size_t iAx=0; iAx<this->jointNames().size(); iAx++)
  {
    report<< this->m_hw->getHandle(this->m_chain.getJointName(iAx)) <<"\n";
  }

  CNR_WARN_COND_THROTTLE(this->m_logger, print_report, throttle_time, report.str() );
  if(!JointController<H,T>::exitUpdate())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->m_logger);
#undef TP
#undef SP
}

template<class H,class T>
inline bool JointCommandController<H,T>::exitStopping()
{
  CNR_TRACE_START(this->m_logger);

  for(unsigned int iAx=0; iAx<this->m_chain.getActiveJointsNumber(); iAx++)
  {
    m_target.q(iAx) = this->getPosition(iAx);
  }
  eigen_utils::setZero(m_target.qd());
  this->m_handler.update(m_target, this->m_chain);

  if(!JointController<H,T>::exitStopping())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }
  m_last_target.copy(m_target, m_target.FULL_STATE);

  CNR_RETURN_TRUE(this->m_logger);
}

template<class H,class T>
inline double JointCommandController<H,T>::getTargetOverride() const
{
  return m_override * m_safe_override_1 * m_safe_override_2;
}

template<class H,class T>
inline void JointCommandController<H,T>::overrideCallback(const std_msgs::Int64ConstPtr& msg)
{
  double ovr;
  if(msg->data > 100)
    ovr = 1;
  else if(msg->data < 0)
    ovr = 0;
  else
    ovr = msg->data * 0.01;
  m_override = ovr;
}

template<class H,class T>
inline void JointCommandController<H,T>::safeOverrideCallback_1(const std_msgs::Int64ConstPtr& msg)
{
  double ovr;
  if(msg->data > 100)
    ovr = 1;
  else if(msg->data < 0)
    ovr = 0;
  else
    ovr = msg->data * 0.01;
  m_safe_override_1 = ovr;
}

template<class H,class T>
inline void JointCommandController<H,T>::safeOverrideCallback_2(const std_msgs::Int64ConstPtr& msg)
{
  double ovr;
  if(msg->data > 100)
    ovr = 1;
  else if(msg->data < 0)
    ovr = 0;
  else
    ovr = msg->data * 0.01;
  m_safe_override_2 = ovr;
}


template<class H,class T>
inline const rosdyn::VectorXd& JointCommandController<H,T>::getCommandPosition( ) const
{
  std::lock_guard<std::mutex> lock(m_mtx);
  return m_target.q();
}

template<class H,class T>
inline const rosdyn::VectorXd& JointCommandController<H,T>::getCommandVelocity( ) const
{
  std::lock_guard<std::mutex> lock(m_mtx);
  return m_target.qd();
}

template<class H,class T>
inline const rosdyn::VectorXd& JointCommandController<H,T>::getCommandAcceleration( ) const
{
  std::lock_guard<std::mutex> lock(m_mtx);
  return m_target.qdd();
}

template<class H,class T>
inline const rosdyn::VectorXd& JointCommandController<H,T>::getCommandEffort( ) const
{
  std::lock_guard<std::mutex> lock(m_mtx);
  return m_target.effort();
}

template<class H,class T>
inline double JointCommandController<H,T>::getCommandPosition(size_t idx) const
{
  std::lock_guard<std::mutex> lock(m_mtx);
  return m_target.q(idx);
}

template<class H,class T>
inline double JointCommandController<H,T>::getCommandVelocity(size_t idx) const
{
  std::lock_guard<std::mutex> lock(m_mtx);
  return m_target.qd(idx);
}

template<class H,class T>
inline double JointCommandController<H,T>::getCommandAcceleration(size_t idx) const
{
  std::lock_guard<std::mutex> lock(m_mtx);
  return m_target.qdd(idx);
}

template<class H,class T>
inline double JointCommandController<H,T>::getCommandEffort(size_t idx) const
{
  std::lock_guard<std::mutex> lock(m_mtx);
  return m_target.effort(idx);
}

template<class H,class T>
inline void JointCommandController<H,T>::setCommandPosition(const rosdyn::VectorXd& in)
{
  std::lock_guard<std::mutex> lock(m_mtx);
  m_target.q() = in;
}

template<class H,class T>
inline void JointCommandController<H,T>::setCommandVelocity(const rosdyn::VectorXd& in)
{
  std::lock_guard<std::mutex> lock(m_mtx);
  m_target.qd()     = in;
}

template<class H,class T>
inline void JointCommandController<H,T>::setCommandAcceleration(const rosdyn::VectorXd& in)
{
  std::lock_guard<std::mutex> lock(m_mtx);
  m_target.qdd()    = in;
}

template<class H,class T>
inline void JointCommandController<H,T>::setCommandEffort(const rosdyn::VectorXd& in)
{
  std::lock_guard<std::mutex> lock(m_mtx);
  m_target.effort() = in;
}

template<class H,class T>
inline void JointCommandController<H,T>::setCommandPosition(const double& in, size_t idx)
{
  std::lock_guard<std::mutex> lock(m_mtx);
  m_target.q(idx) = in;
}

template<class H,class T>
inline void JointCommandController<H,T>::setCommandVelocity(const double& in, size_t idx)
{
  std::lock_guard<std::mutex> lock(m_mtx);
  m_target.qd(idx) = in;
}

template<class H,class T>
inline void JointCommandController<H,T>::setCommandAcceleration(const double& in, size_t idx)
{
  std::lock_guard<std::mutex> lock(m_mtx);
  m_target.qdd(idx) = in;
}

template<class H,class T>
inline void JointCommandController<H,T>::setCommandEffort(const double& in, size_t idx)
{
  std::lock_guard<std::mutex> lock(m_mtx);
  m_target.effort(idx) = in;
}

template<class H,class T>
inline void JointCommandController<H,T>::updateTransformationsThread(int ffwd_kin_type, double hz)
{
  CNR_TRACE_START(this->logger());
  rosdyn::ChainState rstate;
  rosdyn::ChainState target;

  {
    std::lock_guard<std::mutex> lock(this->mtx_);
    if(!rstate.init(this->m_chain))
    {
       CNR_FATAL(this->m_logger, "Chain failure!");
       CNR_RETURN_NOTOK(this->m_logger, void());
    }
    if(!target.init(this->m_chain))
    {
       CNR_FATAL(this->m_logger, "Chain failure!");
       CNR_RETURN_NOTOK(this->m_logger, void());
    }
  }

  ros::Rate rt(hz);
  while(!this->stop_update_transformations_)
  {
    {
      //std::lock_guard<std::mutex> lock(this->mtx_);
      rstate.copy(this->chainState(), this->chainState().ONLY_JOINT);
      target.copy(this->m_target, this->m_target.ONLY_JOINT);
    }
    rstate.updateTransformations(this->m_chain, ffwd_kin_type);
    target.updateTransformations(this->m_chain, ffwd_kin_type);
    {
      //std::lock_guard<std::mutex> lock(this->mtx_);
      this->chainState().copy(rstate, rstate.ONLY_CART);
      this->m_target.copy(target, target.ONLY_CART);
    }
    if(!this->update_transformations_runnig_)
    {
      CNR_INFO(this->logger(), "First state & target update ;)"
                  << "\nstate:\n" << std::to_string(this->chainState())
                    << "\ntarget:\n" << std::to_string(this->m_target));
      this->update_transformations_runnig_ = true;
    }
    rt.sleep();
  }
  CNR_RETURN_OK(this->m_logger, void());
}



}  // namespace control
}  // cnr

#endif
