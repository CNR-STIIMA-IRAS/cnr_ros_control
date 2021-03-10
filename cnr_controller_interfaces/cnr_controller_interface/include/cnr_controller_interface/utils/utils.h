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

#ifndef CNR_CONTROLLER_INTERFACE__UTILS__H
#define CNR_CONTROLLER_INTERFACE__UTILS__H

#include <memory>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <sensor_msgs/JointState.h>


namespace cnr
{
namespace control
{

/**
 * @brief The template is bare class used to cast the boost shared_ptr to std shared_ptr
 */
template<class SharedPointer>
struct Holder
{
  SharedPointer p;

  Holder(const SharedPointer &p) : p(p) {}
  Holder(const Holder &other) : p(other.p) {}
  Holder(Holder &&other) : p(std::move(other.p)) {}

  void operator () (...) { p.reset(); }
};

/**
 * @brief Cast function that uses the 'holder' of the pointer
 */
template<class T>
std::shared_ptr<T> to_std_ptr(const boost::shared_ptr<T> &p)
{
  typedef Holder<std::shared_ptr<T>>   StandardHolder;
  typedef Holder<boost::shared_ptr<T>> BoostHolder;

  StandardHolder *h = boost::get_deleter<StandardHolder>(p);
  if( h )
  {
    return h->p;
  }
  else
  {
    return std::shared_ptr<T>(p.get(), BoostHolder(p));
  }
}





inline bool extractJoint(const sensor_msgs::JointState msg,
  const std::string& name, double* pos = nullptr, double* vel = nullptr, double* eff = nullptr)
{
  if(pos && (msg.position.size()!=msg.name.size()) )
  {
    return false;
  }

  if(vel && (msg.velocity.size()!=msg.name.size()) )
  {
    return false;
  }

  if(eff && (msg.effort.size()!=msg.name.size()) )
  {
    return false;
  }

  std::vector<std::string>::const_iterator it = std::find(msg.name.begin(), msg.name.end(), name);
  if(it == msg.name.end())
  {
    return false;
  }

  size_t iJoint = std::distance(msg.name.begin(), it);
  if(pos) *pos = msg.position.at(iJoint);
  if(vel) *vel = msg.velocity.at(iJoint);
  if(eff) *eff = msg.effort.at(iJoint);

  return true;
}



inline bool extractJoint(const sensor_msgs::JointState msg,
                         const std::vector<std::string>& names,
                         std::vector<double>* pos = nullptr,
                         std::vector<double>* vel = nullptr,
                         std::vector<double>* eff = nullptr)
{
  if(msg.name.size() < names.size())
  {
    return false;
  }

  if(pos && (msg.position.size()!=msg.name.size()) )
  {
    return false;
  }

  if(vel && (msg.velocity.size()!=msg.name.size()) )
  {
    return false;
  }

  if(eff && (msg.effort.size()!=msg.name.size()) )
  {
    return false;
  }

  for(const auto & name : names)
  {
    std::vector<std::string>::const_iterator it = std::find(msg.name.begin(), msg.name.end(), name);
    if(it == msg.name.end())
    {
      return false;
    }

    size_t iJoint = std::distance(msg.name.begin(), it);
    if(pos) pos->at(iJoint) = msg.position.at(iJoint);
    if(vel) vel->at(iJoint) = msg.velocity.at(iJoint);
    if(eff) eff->at(iJoint) = msg.effort.at(iJoint);
  }

  return true;
}



template<typename Derived>
inline bool extractJoint(const sensor_msgs::JointState msg,
                         const std::vector<std::string>& names,
                         Eigen::MatrixBase<Derived>* pos = nullptr,
                         Eigen::MatrixBase<Derived>* vel = nullptr,
                         Eigen::MatrixBase<Derived>* eff = nullptr)
{
  if(msg.name.size() < names.size())
  {
    return false;
  }

  if(pos && (msg.position.size()!=msg.name.size()) )
  {
    return false;
  }

  if(vel && (msg.velocity.size()!=msg.name.size()) )
  {
    return false;
  }

  if(eff && (msg.effort.size()!=msg.name.size()) )
  {
    return false;
  }

  for(const auto & name : names)
  {
    std::vector<std::string>::const_iterator it = std::find(msg.name.begin(), msg.name.end(), name);
    if(it == msg.name.end())
    {
      return false;
    }

    size_t iJoint = std::distance(msg.name.begin(), it);
    if(pos) (*pos)(iJoint) = msg.position.at(iJoint);
    if(vel) (*vel)(iJoint) = msg.velocity.at(iJoint);
    if(eff) (*eff)(iJoint) = msg.effort.at(iJoint);
  }

  return true;
}




}
}

#endif
