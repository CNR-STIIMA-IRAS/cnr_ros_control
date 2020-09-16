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

#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_controller_interface/cnr_controller_interface.h>


#include <cnr_controller_interface/utils/cnr_kinematic_utils.h>

namespace cnr_controller_interface
{



/**
 *
 *
 *
 * Base class to log the controller status
 */
template< class T >
class JointController: public cnr_controller_interface::Controller< T >
{
public:

  ~JointController();

  virtual bool doInit();
  virtual bool doStarting(const ros::Time& /*time*/);
  virtual bool doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/);
  virtual bool doStopping(const ros::Time& /*time*/);
  virtual bool doWaiting(const ros::Time& /*time*/);
  virtual bool doAborting(const ros::Time& /*time*/);

  virtual bool enterInit();
  virtual bool enterStarting();
  virtual bool enterUpdate();

  const size_t& nAx                         ( ) const { return m_kin->nAx(); }
  const Eigen::VectorXd& q                  ( ) const { return m_state->q; }
  const Eigen::VectorXd& qd                 ( ) const { return m_state->qd; }
  const Eigen::VectorXd& qdd                ( ) const { return m_state->qdd; }
  const Eigen::VectorXd& effort             ( ) const { return m_state->effort; }
  const Eigen::VectorXd& upperLimit         ( ) const { return m_kin->upperLimit       (); }
  const Eigen::VectorXd& lowerLimit         ( ) const { return m_kin->lowerLimit       (); }
  const Eigen::VectorXd& speedLimit         ( ) const { return m_kin->speedLimit       (); }
  const Eigen::VectorXd& accelerationLimit  ( ) const { return m_kin->accelerationLimit(); }
  const std::vector<std::string>& jointNames( ) const { return m_kin->jointNames       (); }
  const std::vector<std::string>& linkNames ( ) const { return m_kin->linkNames        (); }
  const double& q                  (size_t iAx) const { return m_state->q(iAx); }
  const double& qd                 (size_t iAx) const { return m_state->qd(iAx); }
  const double& qdd                (size_t iAx) const { return m_state->qdd(iAx); }
  const double& effort             (size_t iAx) const { return m_state->effort(iAx); }
  const double& upperLimit         (size_t iAx) const { return m_kin->upperLimit       (iAx);}
  const double& lowerLimit         (size_t iAx) const { return m_kin->lowerLimit       (iAx);}
  const double& speedLimit         (size_t iAx) const { return m_kin->speedLimit       (iAx);}
  const double& accelerationLimit  (size_t iAx) const { return m_kin->accelerationLimit(iAx);}
  const std::string& jointName     (size_t iAx) const { return m_kin->jointName        (iAx);}

  const std::string& baseLink    ( ) const { return m_kin->baseLink(); }
  const std::string& baseFrame   ( ) const { return baseLink();       }
  const std::string& toolLink    ( ) const { return m_kin->toolLink(); }
  const std::string& toolFrame   ( ) const { return toolLink();       }
  const Eigen::Affine3d& toolPose( ) const { return m_kin->toolPose(); }

protected:

  KinematicsStructPtr m_kin;
  KinematicStatusPtr  m_state;

//  urdf::ModelInterfaceSharedPtr                   m_model;
//  std::string                                     m_base_link;
//  std::string                                     m_tool_link;
//  rosdyn::ChainPtr                                m_chain;
//  Eigen::Affine3d                                 m_Tbt;
//  Eigen::Matrix<double, 6, 1>                     m_twist;
//  Eigen::Matrix6Xd                                m_J;

//private:
//  std::vector<std::string>      m_joint_names;
//  size_t                        m_nAx;

//  Eigen::VectorXd               m_upper_limit;
//  Eigen::VectorXd               m_lower_limit;
//  Eigen::VectorXd               m_qd_limit;
//  Eigen::VectorXd               m_qdd_limit;
};

} // cnr_controller_interface

#include <cnr_controller_interface/cnr_joint_controller_interface_impl.h>

#endif


