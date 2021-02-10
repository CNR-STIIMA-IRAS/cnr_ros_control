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
#include <rosdyn_utilities/chain_state.h>

#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_controller_interface/internal/cnr_handles.h>

#if !defined(MAX_NUM_AXES) || MAX_NUM_AXES==0
  #define MAX_NUM_AXES 20
#endif

namespace cnr
{
namespace control
{

//! This constant is used in the templated-inherited controller,
//! when a dynamic-sized allocation is selected, with pre-allocated memory
constexpr int max_num_axes = MAX_NUM_AXES;

typedef Eigen::Matrix<double,-1,1, Eigen::ColMajor, max_num_axes> Vector;

template<int N, int MaxN=N, std::enable_if_t<N==1,int> =0 >
Vector to(const typename rosdyn::ChainState<N,MaxN>::Value& v);

template<int N, int MaxN=N, std::enable_if_t<N==1,int> =0 >
typename rosdyn::ChainState<N,MaxN>::Value  to(const Vector& v);

template<int N, int MaxN=N, std::enable_if_t<N!=1,int> =0 >
Vector to(const typename rosdyn::ChainState<N,MaxN>::Value& v);

template<int N, int MaxN, std::enable_if_t<N!=1,int> =0 >
typename rosdyn::ChainState<N,MaxN>::Value to(const Vector& v);


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
template<int N, int MaxN, class H, class T>
class JointController: public cnr::control::Controller<T>
{
public:
  virtual ~JointController();

  virtual bool doInit() override;
  virtual bool doStarting(const ros::Time& /*time*/) override;
  virtual bool doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;
  virtual bool doStopping(const ros::Time& /*time*/) override;
  virtual bool doWaiting(const ros::Time& /*time*/) override;
  virtual bool doAborting(const ros::Time& /*time*/) override;

protected:
  virtual bool enterInit() override;
  virtual bool enterStarting() override;
  virtual bool exitStarting() override;
  virtual bool enterUpdate() override;

  // Accessors, to be used by the inherited classes
  const unsigned int& nAx( ) const { return m_chain.getActiveJointsNumber(); }
  const std::vector<std::string>& jointNames( ) const { return m_chain.getActiveJointsName(); }

  Handler<H,T>                   m_handler;
  urdf::ModelInterfaceSharedPtr  m_urdf_model;
  rosdyn::Chain                  m_chain;
  rosdyn::ChainState<N,MaxN>     m_rstate;

  Eigen::IOFormat            m_cfrmt;

  void startUpdateTransformationsThread(int ffwd_kin_type, double hz = 10.0);
  void stopUpdateTransformationsThread();
  virtual void updateTransformationsThread(int ffwd_kin_type, double hz);

  std::thread     update_transformations_;
  bool            stop_update_transformations_;
  std::mutex      mtx_;
};





}  // control
}  // cnr

#include <cnr_controller_interface/internal/cnr_joint_controller_interface_impl.h>

#endif  // CNR_CONTROLLER_INTERFACE__JOINT_CONTROLLER_INTERFACE_H




//  const Eigen::VectorXd&  upperLimit        ( ) const { return m_rkin->upperLimit       (); }
//  const Eigen::VectorXd&  lowerLimit        ( ) const { return m_rkin->lowerLimit       (); }
//  const Eigen::VectorXd&  speedLimit        ( ) const { return m_rkin->speedLimit       (); }
//  const Eigen::VectorXd&  accelerationLimit ( ) const { return m_rkin->accelerationLimit(); }
//  const std::string&      baseLink          ( ) const { return m_rkin->baseLink(); }
//  const std::string&      baseFrame         ( ) const { return baseLink();       }
//  const std::string&      toolLink          ( ) const { return m_rkin->toolLink(); }
//  const std::string&      toolFrame         ( ) const { return toolLink();       }
//  const double& upperLimit         (size_t iAx) const { return m_rkin->upperLimit       (iAx);}
//  const double& lowerLimit         (size_t iAx) const { return m_rkin->lowerLimit       (iAx);}
//  const double& speedLimit         (size_t iAx) const { return m_rkin->speedLimit       (iAx);}
//  const double& accelerationLimit  (size_t iAx) const { return m_rkin->accelerationLimit(iAx);}

//  const std::vector<std::string>& linkNames ( ) const { return m_rkin->linkNames        (); }
//  const std::string& jointName     (size_t iAx) const { return m_rkin->jointName        (iAx);}
//  const std::string& linkName      (size_t iAx) const { return m_rkin->linkNames        (iAx);}
