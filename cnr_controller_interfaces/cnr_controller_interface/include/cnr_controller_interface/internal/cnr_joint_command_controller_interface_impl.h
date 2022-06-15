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
#include <rosdyn_chain_state/chain_state.h>
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
  return cnr::control::JointController<H,T>::doInit();
}

template<class H,class T>
inline bool JointCommandController<H,T>::doStarting(const ros::Time& time)
{
  return cnr::control::JointController<H,T>::doStarting(time);
}

template<class H,class T>
inline bool JointCommandController<H,T>::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  return cnr::control::JointController<H,T>::doUpdate(time,period);
}

template<class H,class T>
inline bool JointCommandController<H,T>::doStopping(const ros::Time& time)
{
  this->stopUpdateTransformationsThread();
  return cnr::control::JointController<H,T>::doStopping(time);
}

template<class H,class T>
inline bool JointCommandController<H,T>::doWaiting(const ros::Time& time)
{
  return cnr::control::JointController<H,T>::doWaiting(time);
}

template<class H,class T>
inline bool JointCommandController<H,T>::doAborting(const ros::Time& time)
{
  return cnr::control::JointController<H,T>::doAborting(time);
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
  {
    std::lock_guard<std::mutex> lock(this->m_target_mtx);
    m_target.init(this->chainNonConst());
    m_last_target.init(this->chainNonConst());
    m_target_threaded.init(this->chainNonConst());
  }

  this->template add_subscriber<std_msgs::Int64>("/speed_ovr" , 1,
                     boost::bind(&JointCommandController<H,T>::overrideCallback, this, _1), false);
  this->template add_subscriber<std_msgs::Int64>("/safe_ovr_1", 1,
                     boost::bind(&JointCommandController<H,T>::safeOverrideCallback_1, this, _1), false);
  this->template add_subscriber<std_msgs::Int64>("/safe_ovr_2", 1,
                 boost::bind(&JointCommandController<H,T>::safeOverrideCallback_2, this, _1), false);

  m_max_velocity_multiplier = 10;
  std::string what;
  rosparam_utilities::get(this->getControllerNamespace() + "/max_velocity_multiplier", m_max_velocity_multiplier, what, &m_max_velocity_multiplier);
  
  m_override = 1;
  m_safe_override_1 = 1;
  m_safe_override_2 = 1;

  bool pub_log_target = false;
  rosparam_utilities::get(this->getControllerNamespace() + "/pub_log_target", pub_log_target, what, &pub_log_target);
  if(pub_log_target)
  {
    m_target_pub.reset(
      new rosdyn::ChainStatePublisher(this->getControllerNh(), this->getControllerNamespace() + "/target",
                                        this->chainNonConst(), &(this->m_target)));
  }
  else
  {
    m_target_pub.reset();
  }

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

  m_target.setZero(this->chainNonConst());
  m_target.q() = this->getPosition();

  {
    std::lock_guard<std::mutex> lock(this->m_target_mtx);
    this->m_handler.update(m_target);
  }

  CNR_DEBUG(this->m_logger, "Target at Start: Position: " << m_target.q().transpose() );
  CNR_DEBUG(this->m_logger, "Target at Start: Velocity: " << m_target.qd().transpose() );
  CNR_DEBUG(this->m_logger, "Target at Start: Effort  : " << m_target.effort().transpose() );

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
  size_t ll = __LINE__;
  try
  {
    ll = __LINE__;
    report << "==========\n";
    report << "Priority           : " << std::to_string(m_priority) << "\n";
    report << "upper limit        : " << TP(this->chain().getQMax()) << "\n";
    report << "lower limit        : " << TP(this->chain().getQMin()) << "\n";
    report << "Speed Limit        : " << TP(this->chain().getDQMax()) << "\n";
    report << "Acceleration Limit : " << TP(this->chain().getDDQMax()) << "\n";
    report << "----------\n";
    ll = __LINE__;
    // ============================== ==============================
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
    ll = __LINE__;

    // ============================== ==============================
    auto saturated_qd = nominal_qd;

    ll = __LINE__;
    if (m_priority != NONE)
    {
      ll = __LINE__;
      if(rosdyn::saturateSpeed(this->chain(), saturated_qd, m_last_target.qd(), m_last_target.q(),
                                 this->m_sampling_period, m_max_velocity_multiplier, true, &report))
      {
        print_report = true;
        m_target.qd() = saturated_qd;
      }
      ll = __LINE__;
      m_target.q()  = m_last_target.q() + saturated_qd * this->m_dt.toSec() +0.5*m_target.qdd()*std::pow(this->m_dt.toSec(),2.0);
    }
    ll = __LINE__;
    m_last_target.copy(m_target, m_target.ONLY_JOINT);

  ll = __LINE__;
    if(m_target_pub)
    {
      m_target_pub->publish();
    }
    ll = __LINE__;
    // ==============================
  }
  catch(std::exception& e)
  {
    CNR_WARN(this->m_logger,"Exception (last executed line: " << ll << "):\n" << e.what());
    m_target.q()  = m_last_target.q();
    eigen_utils::setZero(m_target.qd());
  }
  catch(...)
  {
    CNR_WARN(this->m_logger,"Unhadled (last executed line: " << ll << ")");
    m_target.q()  = m_last_target.q();
    eigen_utils::setZero(m_target.qd());
  }

  report<< "q  trg: " << TP(m_target.q()) << "\n";
  report<< "qd trg: " << TP(m_target.qd()) << "\n";
  report<< "ef trg: " << TP(m_target.effort()) << "\n";

  {
    std::lock_guard<std::mutex> lock(this->m_target_mtx);
    this->m_handler.update(m_target);
  }

  //for(size_t iAx=0; iAx<this->jointNames().size(); iAx++)
  //{
  //  report<< this->m_hw->getHandle(this->chain().getActiveJointName(iAx)) <<"\n";
  //}

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

  for(unsigned int iAx=0; iAx<this->chain().getActiveJointsNumber(); iAx++)
  {
    m_target.q(iAx) = this->getPosition(iAx);
  }
  eigen_utils::setZero(m_target.qd());

  {
    std::lock_guard<std::mutex> lock(this->m_target_mtx);
    this->m_handler.update(m_target);
  }
  
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
inline const rosdyn::ChainState& JointCommandController<H,T>::chainCommand() const
{
  if(this->getKinUpdatePeriod()<=0)
    throw std::runtime_error("The 'kin_update_period' has not been set, and therefore the fkin is not computed!");

  std::lock_guard<std::mutex> lock(this->m_target_mtx);
  return m_target;
}

template<class H,class T>
inline rosdyn::ChainState& JointCommandController<H,T>::chainCommand()
{
  if(this->getKinUpdatePeriod()<=0)
    throw std::runtime_error("The 'kin_update_period' has not been set, and therefore the fkin is not computed!");
  std::lock_guard<std::mutex> lock(this->m_target_mtx);
  return m_target;
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
  ros::Rate rt(hz);
  while(!this->stop_update_transformations_)
  {
    this->m_rstate_threaded.copy(this->chainState(), rosdyn::ChainState::ONLY_JOINT);
    
    {
      std::lock_guard<std::mutex> lock(this->m_target_mtx);
      m_target_threaded.copy(this->m_target, rosdyn::ChainState::ONLY_JOINT);
    }
    this->m_rstate_threaded.updateTransformations(this->m_chain_threaded, ffwd_kin_type);
    m_target_threaded.updateTransformations(this->m_chain_threaded, ffwd_kin_type);
    
    this->chainState().copy(this->m_rstate_threaded, rosdyn::ChainState::ONLY_CART);
    
    {
      std::lock_guard<std::mutex> lock(this->m_target_mtx);
      this->m_target.copy(m_target_threaded, rosdyn::ChainState::ONLY_CART);
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
