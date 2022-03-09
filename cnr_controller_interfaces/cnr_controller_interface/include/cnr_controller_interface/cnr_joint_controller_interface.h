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
#pragma once

#ifndef CNR_CONTROLLER_INTERFACE__JOINT_CONTROLLER_INTERFACE_H
#define CNR_CONTROLLER_INTERFACE__JOINT_CONTROLLER_INTERFACE_H

#include <mutex>
#include <thread>
#include <Eigen/Core>

#include <ros/ros.h>

#include <cnr_logger/cnr_logger.h>
#include <rosdyn_core/primitives.h>
#include <rosdyn_chain_state/chain_state.h>

#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_controller_interface/internal/cnr_handles.h>

namespace cnr
{
namespace control
{

/**
 * @brief The class is designed to get the feedback of a set of joints,
 * and the joints must be connected to each other.
 * The class is built aroun a 'rosdyn::ChainState' that stores the state
 * of the joints, and at each cycle time the internal status is updated.
 * The class creates a parallel thread cyclically (as the sampling rate)
 * compute the forward kinematics. Furthermore, the effort may be computed
 * from the external force measure id available.
 * The computation is therefore done in parallel to avoid that the 'update' method
 * takes too long, breaking the soft-realtime of the controller
 *
 */
template<class H, class T>
class JointController: public cnr::control::Controller<T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~JointController();

  virtual bool doInit() override;
  virtual bool doStarting(const ros::Time& /*time*/) override;
  virtual bool doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;
  virtual bool doStopping(const ros::Time& /*time*/) override;
  virtual bool doWaiting(const ros::Time& /*time*/) override;
  virtual bool doAborting(const ros::Time& /*time*/) override;

  virtual bool enterInit() override;
  virtual bool enterStarting() override;
  virtual bool exitStarting() override;
  virtual bool enterUpdate() override;

protected:
  // Accessors, to be used by the inherited classes
  const unsigned int& nAx( ) const;
  const std::vector<std::string>& jointNames( ) const;

  Handler<H,T>                   m_handler;
  urdf::ModelInterfaceSharedPtr  m_urdf_model;

  const rosdyn::Chain& chain() const;
  rosdyn::Chain& chainNonConst();

  const rosdyn::ChainState& chainState() const;
  rosdyn::ChainState&       chainState();

  const rosdyn::VectorXd& getPosition    ( ) const;
  const rosdyn::VectorXd& getVelocity    ( ) const;
  const rosdyn::VectorXd& getAcceleration( ) const;
  const rosdyn::VectorXd& getEffort      ( ) const;

  double getPosition    (int idx) const;
  double getVelocity    (int idx) const;
  double getAcceleration(int idx) const;
  double getEffort      (int idx) const;

  const Eigen::Affine3d&   getToolPose( ) const;
  const Eigen::Vector6d&   getTwist( ) const;
  const Eigen::Vector6d&   getTwistd( ) const;
  const rosdyn::Matrix6Xd& getJacobian( ) const;

  bool startUpdateTransformationsThread(int ffwd_kin_type, double hz = 10.0);
  void stopUpdateTransformationsThread();
  virtual void updateTransformationsThread(int ffwd_kin_type, double hz);

  std::thread         update_transformations_;
  bool                stop_update_transformations_;
  bool                update_transformations_runnig_;
  mutable std::mutex m_rstate_mtx;
  rosdyn::ChainState m_rstate_threaded;

  double getKinUpdatePeriod() const { return m_fkin_update_period; }
  void setKinUpdatePeriod(const double& fkin_update_period) { m_fkin_update_period = fkin_update_period; }

private:
  rosdyn::LinkPtr    m_root_link;  //link primitivo da cui parte la catena cinematica(world ad esempio)

  mutable std::mutex m_chain_mtx;
  rosdyn::Chain      m_chain; 
  rosdyn::Chain      m_chain_threaded; 

  rosdyn::ChainState m_rstate;
  
  Eigen::IOFormat    m_cfrmt;
  double             m_fkin_update_period;
};

}  // control
}  // cnr

#include <cnr_controller_interface/internal/cnr_joint_controller_interface_impl.h>

#endif  // CNR_CONTROLLER_INTERFACE__JOINT_CONTROLLER_INTERFACE_H

