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
#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>
#include <cnr_controller_interface/utils/cnr_handles_utils.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace cnr_controller_interface
{

template< class T >
JointCommandController< T >::~JointCommandController()
{
  CNR_TRACE_START(*cnr_controller_interface::Controller< T >::m_logger);
}

template< class T >
bool JointCommandController< T >::doInit()
{
  return true;
}

template< class T >
bool JointCommandController<T>::doStarting(const ros::Time& /*time*/)
{
  return true;
}

template< class T >
bool JointCommandController<T>::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  return true;
}

template< class T >
bool JointCommandController<T>::doStopping(const ros::Time& /*time*/)
{
  return true;
}

template< class T >
bool JointCommandController<T>::doWaiting(const ros::Time& /*time*/)
{
  return true;
}

template< class T >
bool JointCommandController<T>::doAborting(const ros::Time& /*time*/)
{
  return true;
}

template< class T >
bool JointCommandController<T>::enterInit()
{
  CNR_TRACE_START(*(this->m_logger));
  if (!JointController<T>::enterInit())
  {
    CNR_RETURN_FALSE(*(this->m_logger));
  }

  m_priority.reset();
  m_target.resize(this->m_kin->jointNames());

  this->template add_subscriber<std_msgs::Int64>("speed_ovr" , "/speed_ovr" , 1,
                      boost::bind(&JointCommandController<T>::overrideCallback, this, _1));
  this->template add_subscriber<std_msgs::Int64>("safe_ovr_1", "/safe_ovr_1", 1,
                      boost::bind(&JointCommandController<T>::safeOverrideCallback_1, this, _1));
  this->template add_subscriber<std_msgs::Int64>("safe_ovr_2", "/safe_ovr_2", 1,
                      boost::bind(&JointCommandController<T>::safeOverrideCallback_2, this, _1));

  m_override = 1;
  m_safe_override_1 = 1;
  m_safe_override_2 = 1;

  CNR_RETURN_BOOL(*(this->m_logger), Controller<T>::dump_state());
}

template< class T >
bool JointCommandController<T>::enterStarting()
{
  CNR_TRACE_START(*(this->m_logger));
  if (!JointController<T>::enterStarting())
  {
    CNR_RETURN_FALSE(*(this->m_logger));
  }

  m_target.q  = this->m_state->q;

  CNR_RETURN_BOOL(*(this->m_logger), Controller<T>::dump_state());
}

template< class T >
bool JointCommandController<T>::enterUpdate()
{
  CNR_TRACE_START_THROTTLE_DEFAULT(*this->m_logger);
  if (!JointController<T>::enterUpdate())
  {
    CNR_RETURN_FALSE(*(this->m_logger));
  }
  m_last_target = m_target;
  CNR_RETURN_BOOL_THROTTLE_DEFAULT(*this->m_logger, Controller<T>::dump_state());
}

template< class T >
bool JointCommandController<T>::exitUpdate()
#define SP std::fixed  << std::setprecision(3)
#define TP(X) std::fixed << std::setprecision(3) << X.format(this->m_cfrmt)
{

  std::stringstream report;
  bool print_report = false;
  double throttle_time = 1.0;
  CNR_TRACE_START_THROTTLE_DEFAULT(*this->m_logger);

  if(!m_priority)
  {
    CNR_RETURN_FALSE(*this->m_logger, "The scaling priority is not set (do it in the intherited function doInit())");
  }
  try
  {
    report << "==========\n";
    report << "Priorty: " << std::to_string(*m_priority) << "\n";
    report << "upper limit        : " << TP(this->m_kin->upperLimit().transpose()) << "\n";
    report << "lower limit        : " << TP(this->m_kin->lowerLimit().transpose()) << "\n";
    report << "Speed Limit        : " << TP(this->m_kin->speedLimit().transpose()) << "\n";
    report << "Acceleration Limit : " << TP(this->m_kin->accelerationLimit().transpose()) << "\n";
    report << "----------\n";
    // ============================== ==============================
    ROS_DEBUG_ONCE("Set Target according to Priority");
    Eigen::VectorXd nominal_qd(this->m_kin->nAx()); nominal_qd.setZero();
    if(*m_priority == Q_PRIORITY)
    {
      if(std::isnan(m_target.q.norm()))
      {
        print_report = true;
        report << "SAFETY CHECK - Received a position with nan values... superimposed to zero!\n";
        m_target.q = m_last_target.q;
      }
      nominal_qd = (m_target.q - m_last_target.q) / this->m_dt.toSec();
    }
    else if(*m_priority == QD_PRIORITY)
    {
      nominal_qd = m_target.qd;
      if(std::isnan(nominal_qd.norm()))
      {
        print_report = true;
        nominal_qd.setZero();
      }
    }
    report << "Nominal qd                  : " << TP(nominal_qd.transpose()) << "\n";
    // ============================== ==============================


    // ============================== ==============================
    Eigen::VectorXd scale(this->m_kin->nAx());
    for(size_t iAx=0;iAx<this->m_kin->nAx();iAx++)
    {
      scale(iAx) = std::fabs(nominal_qd(iAx)) > this->m_kin->speedLimit(iAx)
                 ? this->m_kin->speedLimit(iAx) / std::fabs(nominal_qd(iAx) )
                 : 1.0;
    }
    Eigen::VectorXd saturated_qd = scale.minCoeff() * nominal_qd;
    report << "Saturated qd  [speed limits]: " << TP(saturated_qd.transpose()) << "\n";
    if(scale.minCoeff()  < 1 )
    {
      print_report = true;
      report << "*** Join Velocity Saturation (Max allowed velocity)\n";
    }
    // ==============================

    Eigen::VectorXd qd_dir;
    if(nominal_qd.norm() > 1e-5)
    {
      qd_dir = nominal_qd.normalized();
    }
    else
    {
      qd_dir = (nominal_qd - m_target.qd).normalized();
    }

    Eigen::VectorXd qd_sup = m_last_target.qd + this->m_kin->accelerationLimit() * this->m_dt.toSec();
    Eigen::VectorXd qd_inf = m_last_target.qd - this->m_kin->accelerationLimit() * this->m_dt.toSec();
    Eigen::VectorXd dqd(this->m_kin->nAx());
    for(size_t iAx=0;iAx<this->m_kin->nAx();iAx++)
    {
      dqd (iAx) = saturated_qd(iAx) > qd_sup(iAx) ? (qd_sup(iAx) - saturated_qd(iAx))
                : saturated_qd(iAx) < qd_inf(iAx) ? (qd_inf(iAx) - saturated_qd(iAx))
                : 0.0;
    }
    if(dqd.minCoeff() * dqd.maxCoeff() >= 0.0)
    {
      saturated_qd += (dqd.dot(qd_dir) * qd_dir);
    }
    else
    {
      print_report = true;

      report << "**** The target velocity cannot be reached without the deformation ****\n";
      report << "dqd            : " << TP(saturated_qd.transpose()) <<"\n";
      report << "dqd min        : " << dqd.minCoeff() << " max: " << dqd.maxCoeff() <<"\n";
      report << "Prev target vel: " << TP(m_target.qd.transpose()) << "\n";
      report << "qd_sup         : " << TP(qd_sup.transpose()) << "\n";
      report << "qd_inf         : " << TP(qd_inf.transpose()) << "\n";
      report << "Calc correction: " << TP(dqd.transpose()) << "\n";
      saturated_qd += dqd;
    }
    report << "Saturated vel [acc limits] : " << TP(saturated_qd.transpose()) << "\n";

    Eigen::VectorXd q_saturated_qd = m_last_target.q + saturated_qd* this->m_dt.toSec();
    Eigen::VectorXd braking_distance(this->nAx());
    for(size_t iAx=0; iAx<this->m_kin->nAx();iAx++)
    {
      braking_distance(iAx)  = 0.5 * this->m_kin->accelerationLimit(iAx)
                               * std::pow(std::abs(saturated_qd(iAx))/this->m_kin->accelerationLimit(iAx) , 2.0);
    }
    report << "q_saturated_qd  : " << TP(q_saturated_qd.transpose()) << "\n";
    report << "braking_distance: " << TP(braking_distance.transpose()) << "\n";

    for(size_t iAx=0; iAx<this->m_kin->nAx();iAx++)
    {
      if ((q_saturated_qd(iAx) > (this->m_kin->upperLimit(iAx) - braking_distance(iAx))) && (saturated_qd(iAx)>0))
      {
        print_report = true;
        report << "**** Brake! Maximum limit approaching on '" <<  this->m_kin->jointName(iAx) << "' ****\n";
        saturated_qd(iAx) = std::max(0.0, saturated_qd(iAx) - this->m_kin->accelerationLimit(iAx) * this->m_dt.toSec());
      }
      else if((q_saturated_qd(iAx)<(this->m_kin->lowerLimit(iAx) + braking_distance(iAx))) && (saturated_qd(iAx)<0))
      {
        print_report = true;
        report << "**** Brake! Minimum limit approaching on '" <<  this->m_kin->jointName(iAx) << "' ****\n";
        saturated_qd(iAx) = std::min(0.0, saturated_qd(iAx) + this->m_kin->accelerationLimit(iAx) * this->m_dt.toSec());
      }
    }
    report << "Saturated vel [braking] : " << TP(saturated_qd.transpose()) << "\n";
    m_target.qd  = saturated_qd;
    m_target.q   = m_last_target.q + m_target.qd * this->m_dt.toSec();
    // ==============================


  }
  catch(...)
  {
    CNR_WARN(*this->logger(),"something wrong in JointTargetFilter::update");
    m_target.q  = m_last_target.q;
    m_target.qd.setZero();
  }

  report<< "q  trg: " << TP(m_target.q.transpose()) << "\n";
  report<< "qd trg: " << TP(m_target.qd.transpose()) << "\n";
  report<< "ef trg: " << TP(m_target.effort.transpose()) << "\n";

  if(!set_to_hw(m_target, this->m_hw))
  {
    CNR_RETURN_FALSE(*(this->m_logger), "Error in download the data to the HW.");
  }

  for (size_t iAx = 0; iAx<this->m_hw->getNames().size(); iAx++)
  {
    report<< this->m_hw->getHandle(this->m_hw->getNames().at(iAx)) <<"\n";
  }

  CNR_WARN_COND_THROTTLE(this->logger(), true, throttle_time, report.str() );


  if (!JointController<T>::exitUpdate())
  {
    CNR_RETURN_FALSE(*(this->m_logger));
  }

  CNR_RETURN_BOOL_THROTTLE_DEFAULT(*(this->m_logger), Controller<T>::dump_state());
#undef TP
#undef SP
}

template< class T >
bool JointCommandController<T>::exitStopping()
{
  CNR_TRACE_START(*(this->m_logger));

  for (unsigned int iAx=0; iAx<this->m_kin->nAx(); iAx++)
  {
    m_target.q(iAx) = this->m_state->q(iAx);
  }
  m_target.qd.setZero();
  set_to_hw(m_target, this->m_hw);

  if (!JointController<T>::exitUpdate())
  {
    CNR_RETURN_FALSE(*(this->m_logger));
  }

  m_last_target = m_target;

  CNR_RETURN_BOOL(*(this->m_logger), Controller<T>::dump_state());
}

template<class T>
double JointCommandController<T>::getTargetOverride() const
{
  return m_override * m_safe_override_1 * m_safe_override_2;
}

template< class T>
void JointCommandController<T>::overrideCallback(const std_msgs::Int64ConstPtr& msg)
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

template< class T>
void JointCommandController<T>::safeOverrideCallback_1(const std_msgs::Int64ConstPtr& msg)
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

template< class T>
void JointCommandController<T>::safeOverrideCallback_2(const std_msgs::Int64ConstPtr& msg)
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
