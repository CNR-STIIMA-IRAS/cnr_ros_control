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
#ifndef CNR_CONTROLLER_INTERFACE__JOINT_CONTROLLER_INTERFACE_H
#define CNR_CONTROLLER_INTERFACE__JOINT_CONTROLLER_INTERFACE_H

#include <mutex>
#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_controller_interface/internal/cnr_handles.h>
#include <rosdyn_utilities/chain_state.h>

namespace cnr_controller_interface
{



/**
 *
 *
 *
 * Base class to log the controller status
 */
template<class H, class T>
class JointController: public cnr_controller_interface::Controller<T>
{
public:

  ~JointController();

  virtual bool doInit() override;
  virtual bool doStarting(const ros::Time& /*time*/) override;
  virtual bool doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;
  virtual bool doStopping(const ros::Time& /*time*/) override;
  virtual bool doWaiting(const ros::Time& /*time*/) override;
  virtual bool doAborting(const ros::Time& /*time*/) override;

  virtual bool enterInit() override;
  virtual bool enterStarting() override;
  virtual bool enterUpdate() override;

  const Eigen::VectorXd& q                  ( ) const { return m_rstate->q(); }
  const Eigen::VectorXd& qd                 ( ) const { return m_rstate->qd(); }
  const Eigen::VectorXd& qdd                ( ) const { return m_rstate->qdd(); }
  const Eigen::VectorXd& effort             ( ) const { return m_rstate->effort(); }
  const double& q                  (size_t iAx) const { return m_rstate->q(iAx); }
  const double& qd                 (size_t iAx) const { return m_rstate->qd(iAx); }
  const double& qdd                (size_t iAx) const { return m_rstate->qdd(iAx); }
  const double& effort             (size_t iAx) const { return m_rstate->effort(iAx); }

  const size_t& nAx                         ( ) const { return m_rkin->nAx(); }
  const Eigen::VectorXd&  upperLimit        ( ) const { return m_rkin->upperLimit       (); }
  const Eigen::VectorXd&  lowerLimit        ( ) const { return m_rkin->lowerLimit       (); }
  const Eigen::VectorXd&  speedLimit        ( ) const { return m_rkin->speedLimit       (); }
  const Eigen::VectorXd&  accelerationLimit ( ) const { return m_rkin->accelerationLimit(); }
  const std::string&      baseLink          ( ) const { return m_rkin->baseLink(); }
  const std::string&      baseFrame         ( ) const { return baseLink();       }
  const std::string&      toolLink          ( ) const { return m_rkin->toolLink(); }
  const std::string&      toolFrame         ( ) const { return toolLink();       }
  const Eigen::Affine3d&  toolPose          ( ) const { return m_rstate->toolPose(); }
  const double& upperLimit         (size_t iAx) const { return m_rkin->upperLimit       (iAx);}
  const double& lowerLimit         (size_t iAx) const { return m_rkin->lowerLimit       (iAx);}
  const double& speedLimit         (size_t iAx) const { return m_rkin->speedLimit       (iAx);}
  const double& accelerationLimit  (size_t iAx) const { return m_rkin->accelerationLimit(iAx);}
  const std::vector<std::string>& jointNames( ) const { return m_rkin->jointNames       (); }
  const std::vector<std::string>& linkNames ( ) const { return m_rkin->linkNames        (); }
  const std::string& jointName     (size_t iAx) const { return m_rkin->jointName        (iAx);}
  const std::string& linkName      (size_t iAx) const { return m_rkin->linkNames        (iAx);} 
protected:
  
  Handler<H,T>              m_handler; 
  rosdyn::ChainInterfacePtr m_rkin;
  rosdyn::ChainStatePtr     m_rstate;

  std::mutex                m_mtx;
  Eigen::IOFormat           m_cfrmt;
};

} // cnr_controller_interface

#include <cnr_controller_interface/internal/cnr_joint_controller_interface_impl.h>

#endif  // CNR_CONTROLLER_INTERFACE__JOINT_CONTROLLER_INTERFACE_H


