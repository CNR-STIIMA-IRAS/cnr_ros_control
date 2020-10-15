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
#ifndef CNR_CONTOLLER_INTERFACE__CNR_JOINT_COMMAND_CONTROLLER_INTERFACE_IMPL_H
#define CNR_CONTOLLER_INTERFACE__CNR_JOINT_COMMAND_CONTROLLER_INTERFACE_IMPL_H

#include <std_msgs/Int64.h>
#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>
#include <rosdyn_core/chain_state.h>
#include <cnr_controller_interface/internal/cnr_handles.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace cnr_controller_interface
{

template<class H, class T>
JointCommandController<H,T>::~JointCommandController()
{
  CNR_TRACE_START(*cnr_controller_interface::Controller< T >::m_logger);
}

template<class H, class T>
bool JointCommandController<H,T>::doInit()
{
  return true;
}

template<class H, class T>
bool JointCommandController<H,T>::doStarting(const ros::Time& /*time*/)
{
  return true;
}

template<class H, class T>
bool JointCommandController<H,T>::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  return true;
}

template<class H, class T>
bool JointCommandController<H,T>::doStopping(const ros::Time& /*time*/)
{
  return true;
}

template<class H, class T>
bool JointCommandController<H,T>::doWaiting(const ros::Time& /*time*/)
{
  return true;
}

template<class H, class T>
bool JointCommandController<H,T>::doAborting(const ros::Time& /*time*/)
{
  return true;
}

template<class H, class T>
bool JointCommandController<H,T>::enterInit()
{
  CNR_TRACE_START(this->m_logger);
  if (!JointController<H,T>::enterInit())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  m_priority = QD_PRIORITY;
  m_target.reset(new rosdyn::ChainState(this->m_rkin));
  m_last_target.reset(new rosdyn::ChainState(this->m_rkin));

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

template<class H, class T>
bool JointCommandController<H,T>::enterStarting()
{
  CNR_TRACE_START(this->m_logger);
  if (!JointController<H,T>::enterStarting())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  m_target->setZero();
  m_target->q() = this->m_rstate->q();
  m_target->updateTransformation();
  
  CNR_RETURN_TRUE(this->m_logger);
}

template<class H, class T>
bool JointCommandController<H,T>::enterUpdate()
{
  CNR_TRACE_START_THROTTLE_DEFAULT(*this->m_logger);
  if (!JointController<H,T>::enterUpdate())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }
  *m_last_target = *m_target;
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(*this->m_logger);
}

template<class H, class T>
bool JointCommandController<H,T>::exitUpdate()
#define SP std::fixed  << std::setprecision(5)
#define TP(X) std::fixed << std::setprecision(5) << X.format(this->m_cfrmt)
{
  std::stringstream report;
  bool print_report = false;
  double throttle_time = 1.0;
  CNR_TRACE_START_THROTTLE_DEFAULT(*this->m_logger);

  try
  {
    report << "==========\n";
    report << "Priorty            : " << std::to_string(m_priority) << "\n";
    report << "upper limit        : " << TP(this->m_rkin->upperLimit().transpose()) << "\n";
    report << "lower limit        : " << TP(this->m_rkin->lowerLimit().transpose()) << "\n";
    report << "Speed Limit        : " << TP(this->m_rkin->speedLimit().transpose()) << "\n";
    report << "Acceleration Limit : " << TP(this->m_rkin->accelerationLimit().transpose()) << "\n";
    report << "----------\n";
    // ============================== ==============================
    ROS_DEBUG_ONCE("Set Target according to Priority");
    Eigen::VectorXd nominal_qd(this->m_rkin->nAx()); nominal_qd.setZero();
    if(m_priority == Q_PRIORITY)
    {
      if(std::isnan(m_target->q().norm()))
      {
        print_report = true;
        report << "SAFETY CHECK - Received a position with nan values... superimposed to zero!\n";
        m_target->q() = m_last_target->q();
      }
      nominal_qd = (m_target->q() - m_last_target->q()) / this->m_dt.toSec();
    }
    else if(m_priority == QD_PRIORITY)
    {
      nominal_qd = m_target->qd();
      if(std::isnan(nominal_qd.norm()))
      {
        print_report = true;
        nominal_qd.setZero();
      }
    }
    report << "Nominal command qd (input)  : " << TP(nominal_qd.transpose()) << "\n";
    // ============================== ==============================


    // ============================== ==============================
    Eigen::VectorXd saturated_qd = nominal_qd;
    
    if(this->m_rkin->saturateSpeed(saturated_qd, this->qd(), this->q(), 
                               this->m_sampling_period, m_max_velocity_multiplier, true, &report))
    {
      print_report = true;
    }
    // ==============================
  }
  catch(...)
  {
    CNR_WARN(*this->logger(),"something wrong in JointTargetFilter::update");
    m_target->q()  = m_last_target->q();
    m_target->qd().setZero();
  }

  report<< "q  trg: " << TP(m_target->q().transpose()) << "\n";
  report<< "qd trg: " << TP(m_target->qd().transpose()) << "\n";
  report<< "ef trg: " << TP(m_target->effort().transpose()) << "\n";

  this->m_handler << m_target;

  for (size_t iAx = 0; iAx<this->jointNames().size(); iAx++)
  {
    report<< this->m_hw->getHandle(this->jointName(iAx)) <<"\n";
  }

  CNR_WARN_COND_THROTTLE(this->logger(), print_report, throttle_time, report.str() );
  if (!JointController<H,T>::exitUpdate())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->m_logger);
#undef TP
#undef SP
}

template<class H, class T>
bool JointCommandController<H,T>::exitStopping()
{
  CNR_TRACE_START(this->m_logger);

  for (unsigned int iAx=0; iAx<this->m_rkin->nAx(); iAx++)
  {
    m_target->q(iAx) = this->m_rstate->q(iAx);
  }
  m_target->qd().setZero();
  this->m_handler << m_target;

  if (!JointController<H,T>::exitStopping())
  {
    CNR_RETURN_FALSE(this->m_logger);
  }
  m_last_target = m_target;

  CNR_RETURN_TRUE(this->m_logger);
}

template<class H, class T>
double JointCommandController<H,T>::getTargetOverride() const
{
  return m_override * m_safe_override_1 * m_safe_override_2;
}

template<class H, class T>
void JointCommandController<H,T>::overrideCallback(const std_msgs::Int64ConstPtr& msg)
{
  double ovr;
  if (msg->data > 100)
    ovr = 1;
  else if (msg->data < 0)
    ovr = 0;
  else
    ovr = msg->data * 0.01;
  m_override = ovr;
}

template<class H, class T>
void JointCommandController<H,T>::safeOverrideCallback_1(const std_msgs::Int64ConstPtr& msg)
{
  double ovr;
  if (msg->data > 100)
    ovr = 1;
  else if (msg->data < 0)
    ovr = 0;
  else
    ovr = msg->data * 0.01;
  m_safe_override_1 = ovr;
}

template<class H, class T>
void JointCommandController<H,T>::safeOverrideCallback_2(const std_msgs::Int64ConstPtr& msg)
{
  double ovr;
  if (msg->data > 100)
    ovr = 1;
  else if (msg->data < 0)
    ovr = 0;
  else
    ovr = msg->data * 0.01;
  m_safe_override_2 = ovr;
}

} // cnr_controller_interface
#endif
