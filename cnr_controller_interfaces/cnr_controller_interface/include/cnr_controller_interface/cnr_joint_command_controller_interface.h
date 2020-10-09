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
#ifndef CNR_CONTROLLER_INTERFACE__JOINT_COMMAND_CONTROLLER_INTERFACE_H
#define CNR_CONTROLLER_INTERFACE__JOINT_COMMAND_CONTROLLER_INTERFACE_H

#include <mutex>
#include <Eigen/Core>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>
#include <cnr_controller_interface/cnr_joint_controller_interface.h>

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
template<class H, class T>
class JointCommandController: public cnr_controller_interface::JointController<H,T>
{
public:

  enum InputType { Q_PRIORITY, QD_PRIORITY };
  ~JointCommandController();

  virtual bool doInit();
  virtual bool doStarting(const ros::Time& /*time*/);
  virtual bool doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/);
  virtual bool doStopping(const ros::Time& /*time*/);
  virtual bool doWaiting(const ros::Time& /*time*/);
  virtual bool doAborting(const ros::Time& /*time*/);

  virtual bool enterInit();
  virtual bool enterStarting();
  virtual bool enterUpdate();
  virtual bool exitUpdate();
  virtual bool exitStopping();

  const Eigen::VectorXd& getCommandPosition    ( ) { std::lock_guard<std::mutex> lock( m_mtx); return m_target->q(); }
  const Eigen::VectorXd& getCommandVelocity    ( ) { std::lock_guard<std::mutex> lock( m_mtx); return m_target->qd(); }
  const Eigen::VectorXd& getCommandAcceleration( ) { std::lock_guard<std::mutex> lock( m_mtx); return m_target->qdd(); }
  const Eigen::VectorXd& getCommandEffort      ( ) { std::lock_guard<std::mutex> lock( m_mtx); return m_target->effort();}

  double getCommandPosition    (size_t idx) { std::lock_guard<std::mutex> lock( m_mtx); return m_target->q     (idx);}
  double getCommandVelocity    (size_t idx) { std::lock_guard<std::mutex> lock( m_mtx); return m_target->qd    (idx);}
  double getCommandAcceleration(size_t idx) { std::lock_guard<std::mutex> lock( m_mtx); return m_target->qdd   (idx);}
  double getCommandEffort      (size_t idx) { std::lock_guard<std::mutex> lock( m_mtx); return m_target->effort(idx);}

  void setCommandPosition     (const Eigen::VectorXd& in) { std::lock_guard<std::mutex> lock( m_mtx); m_target->q()      = in; }
  void setCommandVelocity     (const Eigen::VectorXd& in) { std::lock_guard<std::mutex> lock( m_mtx); m_target->qd()     = in; }
  void setCommandAcceleration (const Eigen::VectorXd& in) { std::lock_guard<std::mutex> lock( m_mtx); m_target->qdd()    = in; }
  void setCommandEffort       (const Eigen::VectorXd& in) { std::lock_guard<std::mutex> lock( m_mtx); m_target->effort() = in; }

  void setCommandPosition     (const double& in, size_t idx) { std::lock_guard<std::mutex> lock( m_mtx); m_target->q      (idx) = in; }
  void setCommandVelocity     (const double& in, size_t idx) { std::lock_guard<std::mutex> lock( m_mtx); m_target->qd     (idx) = in; }
  void setCommandAcceleration (const double& in, size_t idx) { std::lock_guard<std::mutex> lock( m_mtx); m_target->qdd    (idx) = in; }
  void setCommandEffort       (const double& in, size_t idx) { std::lock_guard<std::mutex> lock( m_mtx); m_target->effort (idx) = in; }

  virtual double getTargetOverride() const;

  void setPriority( const InputType& priority ) { m_priority = priority; }
  
protected:
  std::mutex      m_mtx;
  
private:
  
  InputType             m_priority;
  KinematicStatusPtr    m_target;
  KinematicStatusPtr    m_last_target;

  double m_override;
  double m_safe_override_1;
  double m_safe_override_2;
  double m_max_velocity_multiplier;

  void overrideCallback(const std_msgs::Int64ConstPtr& msg);
  void safeOverrideCallback_1(const std_msgs::Int64ConstPtr& msg);
  void safeOverrideCallback_2(const std_msgs::Int64ConstPtr& msg);

};

} // cnr_controller_interface

#include <cnr_controller_interface/internal/cnr_joint_command_controller_interface_impl.h>

#endif


