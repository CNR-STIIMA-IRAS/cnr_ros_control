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
#include <sensor_msgs/JointState.h>

#include <cnr_logger/cnr_logger.h>
#include <rosdyn_chain_state/chain_state.h>
#include <rosdyn_chain_state/chain_state_publisher.h>
#include <cnr_controller_interface/cnr_joint_controller_interface.h>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>


namespace cnr
{
namespace control
{

/**
 *
 *
 *
 * Base class to log the controller status
 */
template<class H, class T>
class JointCommandController: public cnr::control::JointController<H,T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum InputType {Q_PRIORITY, QD_PRIORITY, NONE};

  JointCommandController() = default;
  virtual ~JointCommandController();

  virtual bool doInit() override;
  virtual bool doStarting(const ros::Time& time) override;
  virtual bool doUpdate(const ros::Time& time, const ros::Duration& period) override;
  virtual bool doStopping(const ros::Time& time) override;
  virtual bool doWaiting(const ros::Time& time) override;
  virtual bool doAborting(const ros::Time& time) override;

  virtual bool enterInit() override;
  virtual bool enterStarting() override;
  virtual bool enterUpdate() override;
  virtual bool exitUpdate() override;
  virtual bool exitStopping() override;

protected:
  const rosdyn::ChainState& chainCommand() const;
  rosdyn::ChainState&       chainCommand();

  const rosdyn::VectorXd& getCommandPosition    ( ) const;
  const rosdyn::VectorXd& getCommandVelocity    ( ) const;
  const rosdyn::VectorXd& getCommandAcceleration( ) const;
  const rosdyn::VectorXd& getCommandEffort      ( ) const;

  double getCommandPosition    (size_t idx) const;
  double getCommandVelocity    (size_t idx) const;
  double getCommandAcceleration(size_t idx) const;
  double getCommandEffort      (size_t idx) const;

  void setCommandPosition     (const rosdyn::VectorXd& in);
  void setCommandVelocity     (const rosdyn::VectorXd& in);
  void setCommandAcceleration (const rosdyn::VectorXd& in);
  void setCommandEffort       (const rosdyn::VectorXd& in);

  void setCommandPosition     (const double& in, size_t idx);
  void setCommandVelocity     (const double& in, size_t idx);
  void setCommandAcceleration (const double& in, size_t idx);
  void setCommandEffort       (const double& in, size_t idx);

  virtual double getTargetOverride() const;

  void setPriority( const InputType& priority ) { m_priority = priority; }

  mutable std::mutex m_mtx;

private:
  InputType          m_priority;
  mutable std::mutex m_target_mtx;
  rosdyn::ChainState m_target;
  rosdyn::ChainState m_target_threaded;
  
  rosdyn::ChainState m_last_target;
  rosdyn::ChainStatePublisherPtr m_target_pub;
  

  double m_override;
  void overrideCallback(const std_msgs::Int64ConstPtr& msg);

  double m_safe_override_1;
  double m_safe_override_2;
  double m_max_velocity_multiplier;
  void safeOverrideCallback_1(const std_msgs::Int64ConstPtr& msg);
  void safeOverrideCallback_2(const std_msgs::Int64ConstPtr& msg);

  virtual void updateTransformationsThread(int ffwd_kin_type, double hz) override;
};

}  // namespace control
}  // namespace cnr

#include <cnr_controller_interface/internal/cnr_joint_command_controller_interface_impl.h>

#endif  // CNR_CONTROLLER_INTERFACE__JOINT_COMMAND_CONTROLLER_INTERFACE_H


