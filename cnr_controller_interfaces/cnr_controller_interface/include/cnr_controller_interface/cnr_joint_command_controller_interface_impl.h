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
  m_target.resize(this->m_kin->nAx());

  this->template add_subscriber<std_msgs::Int64>("speed_ovr" , "/speed_ovr" , 1,
                      boost::bind(&JointCommandController<T>::overrideCallback, this, _1) );
  this->template add_subscriber<std_msgs::Int64>("safe_ovr_1", "/safe_ovr_1", 1,
                      boost::bind(&JointCommandController<T>::safeOverrideCallback_1, this, _1) );
  this->template add_subscriber<std_msgs::Int64>("safe_ovr_2", "/safe_ovr_2", 1,
                      boost::bind(&JointCommandController<T>::safeOverrideCallback_2, this, _1) );

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
  CNR_TRACE_START(*(this->m_logger));
  if (!JointController<T>::enterUpdate())
  {
    CNR_RETURN_FALSE(*(this->m_logger));
  }

  m_last_target = m_target;

  CNR_RETURN_BOOL(*(this->m_logger), Controller<T>::dump_state());
}

template< class T >
bool JointCommandController<T>::exitUpdate()
{
  CNR_TRACE_START(*(this->m_logger));

  if(!m_priority)
  {
    CNR_RETURN_FALSE(*(this->m_logger), "The scaling priority is not set (to do in the template intherited function doInit())");
  }
  try
  {
    // ============================== ==============================
    ROS_DEBUG_ONCE( "[ JointTargetKinematicFilter ] Set Target according to Priority");
    Eigen::VectorXd nominal_qd(this->m_kin->nAx()); nominal_qd.setZero();
    if( *m_priority == Q_PRIORITY )
    {
      if(std::isnan(m_target.q.norm()))
      {
        ROS_WARN_THROTTLE( 2, "[ JointTargetKinematicFilter ] SAFETY CHECK - Received a position with nan values... superimposed to zero!");
        m_target.q = m_last_target.q;
      }
      nominal_qd = ( m_target.q - m_last_target.q ) / this->m_dt.toSec();
    }
    else if( *m_priority == QD_PRIORITY )
    {
      ROS_DEBUG_ONCE( "[ JointTargetKinematicFilter ] Velocity Priority");
      nominal_qd = m_target.qd;
      if(std::isnan(nominal_qd.norm()))
      {
        ROS_WARN_THROTTLE( 2, "[ JointTargetKinematicFilter ] SAFETY CHECK - Received a velocity with nan values... superimposed to zero!");
        nominal_qd.setZero();
      }
    }
    // ============================== ==============================


    // ==============================
    Eigen::VectorXd scale( this->m_kin->nAx() );
    for(size_t iAx=0;iAx<this->m_kin->nAx();iAx++)
    {
      scale(iAx) = std::fabs( nominal_qd(iAx) ) > this->m_kin->speedLimit(iAx)
                 ?  this->m_kin->speedLimit(iAx) / std::fabs( nominal_qd(iAx)  )
                 : 1.0;
    }
    Eigen::VectorXd saturated_qd = scale.minCoeff() * nominal_qd;
    if( scale.minCoeff()  < 1  )
    {
      ROS_WARN_THROTTLE(2,"Join Velocity Saturation (Max allowed velocity)");
    }
    // ==============================

    Eigen::VectorXd qd_dir;
    if( nominal_qd.norm() > 1e-5 )
    {
      qd_dir = nominal_qd.normalized();
    }
    else
    {
      qd_dir = ( nominal_qd - m_target.qd ).normalized();
    }

    Eigen::VectorXd qd_sup = m_target.qd + this->m_kin->accelerationLimit() * this->m_dt.toSec();
    Eigen::VectorXd qd_inf = m_target.qd - this->m_kin->accelerationLimit() * this->m_dt.toSec();
    Eigen::VectorXd dqd(this->m_kin->nAx());
    for(size_t iAx=0;iAx<this->m_kin->nAx();iAx++)
    {
      dqd (iAx) = saturated_qd(iAx) > qd_sup(iAx) ? ( qd_sup(iAx) - saturated_qd(iAx) )
                : saturated_qd(iAx) < qd_inf(iAx) ? ( qd_inf(iAx) - saturated_qd(iAx) )
                : 0.0;
    }
    if(dqd.minCoeff() * dqd.maxCoeff() >= 0.0)
    {
      saturated_qd += ( dqd.dot( qd_dir ) * qd_dir );
    }
    else
    {
      ROS_FATAL_STREAM_THROTTLE(1, "The target velocity cannot be reached without the deformation of the trajectory... (min: : " << dqd.minCoeff() << " max: " << dqd.maxCoeff() <<")" );
      ROS_FATAL_STREAM("Prev target vel: " << m_target.qd.transpose()  );
      ROS_FATAL_STREAM("qd_sup         : " << qd_sup.transpose()  );
      ROS_FATAL_STREAM("qd_inf         : " << qd_inf.transpose()  );
      ROS_FATAL_STREAM("Requested vel  : " << saturated_qd.transpose()  );
      ROS_FATAL_STREAM("Calc correction: " << dqd.transpose()  );
      saturated_qd += dqd;
    }

    Eigen::VectorXd q_saturated_qd = m_last_target.q + saturated_qd* this->m_dt.toSec();
    for(size_t iAx=0; iAx<this->m_kin->nAx();iAx++)
    {
      ROS_DEBUG_STREAM_ONCE( "[ JointTargetKinematicFilter ] Calc brake distance" << saturated_qd(iAx) );;
      double braking_distance  = 0.5 * this->m_kin->accelerationLimit(iAx)
                               * std::pow( std::abs( saturated_qd(iAx) )/this->m_kin->accelerationLimit(iAx) , 2.0);

      if ( (q_saturated_qd(iAx) > (this->m_kin->upperLimit(iAx) - braking_distance) ) && ( saturated_qd(iAx) > 0) )
      {
        ROS_WARN_THROTTLE(2,"[ JointTargetKinematicFilter ] Breaking, maximum limit approaching on joint %s",
                          this->m_kin->jointName(iAx).c_str());
        saturated_qd(iAx) = std::max(0.0, saturated_qd(iAx) - this->m_kin->accelerationLimit(iAx) * this->m_dt.toSec());
      }
      else if( (q_saturated_qd(iAx) < (this->m_kin->lowerLimit(iAx) + braking_distance)) && ( saturated_qd(iAx) < 0) )
      {
        ROS_WARN_THROTTLE(2,"[ JointTargetKinematicFilter ] Breaking, minimum limit approaching on joint %s",
                          this->m_kin->jointName(iAx).c_str());
        saturated_qd(iAx) = std::min(0.0, saturated_qd(iAx) + this->m_kin->accelerationLimit(iAx) * this->m_dt.toSec());
      }
      ROS_DEBUG_STREAM_ONCE( "[ JointTargetKinematicFilter ] Calc scaling value" );
    }

    m_target.qd  = saturated_qd;
    m_target.q   = m_last_target.q + m_target.qd * this->m_dt.toSec();
    // ==============================

  }
  catch(...)
  {
    ROS_WARN("[ JointTargetKinematicFilter ] something wrong in JointTargetFilter::update" );
    m_target.q  = m_last_target.q;
    m_target.qd.setZero();
  }

  extract< T >(m_target, this->m_hw);

  if (!JointController<T>::exitUpdate())
  {
    CNR_RETURN_FALSE(*(this->m_logger));
  }

  CNR_RETURN_BOOL(*(this->m_logger), Controller<T>::dump_state());
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
  extract< T >(m_target, this->m_hw);

  if (!JointController<T>::exitUpdate())
  {
    CNR_RETURN_FALSE(*(this->m_logger));
  }

  m_last_target = m_target;

  CNR_RETURN_BOOL(*(this->m_logger), Controller<T>::dump_state());
}


template<class T>
double JointCommandController<T>::generalOverride() const
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
