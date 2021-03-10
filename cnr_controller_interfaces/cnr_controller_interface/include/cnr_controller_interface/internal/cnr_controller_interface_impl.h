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

#pragma once  //workoround for bug of https://bugreports.qt.io/browse/QTCREATORBUG-20883

#ifndef CNR_CONTROLLER_INTFERFACE_CNR_CONTROLLER_INTFERFACE_IMPL_H
#define CNR_CONTROLLER_INTFERFACE_CNR_CONTROLLER_INTFERFACE_IMPL_H

#include <stdexcept>
#include <mutex>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>

#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_controller_interface_params/cnr_controller_interface_params.h>

namespace cnr
{
namespace control
{


template<class T>
Controller<T>::~Controller()
{
  CNR_TRACE_START(m_logger);
  shutdown("UNLOADED");
  CNR_TRACE(m_logger, "[ DONE]");
}

/**
*
*
*
* Base class to log the controller status
*/
template<class T>
bool Controller<T>::init(T* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  size_t l = __LINE__;
  if(!hw)
  {
    std::cerr << cnr_logger::RED() << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " ;
    std::cerr << "The argument 'hw' is a nullptr. " << std::endl;
    return false;
  }
  try
  {
    l = __LINE__;
    m_hw_name   = root_nh.getNamespace();
    m_ctrl_name = controller_nh.getNamespace();
    if(m_ctrl_name.find(m_hw_name) != std::string::npos)
    {
      m_ctrl_name.erase(m_ctrl_name.find(m_hw_name), m_hw_name.length());
    }
    else
    {
      std::cerr << cnr_logger::RED() << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " ;
      std::cerr << "Controller configurations seems broken. " << std::endl;
      std::cerr << " root ns (the namespace of the hw): " << root_nh.getNamespace() << std::endl;
      std::cerr << " controller ns (the namespace of the ctrl): " << controller_nh.getNamespace() << cnr_logger::RST()
                << std::endl;
      return false;
    }
    l = __LINE__;
    std::replace(m_hw_name.begin(), m_hw_name.end(), '/', '_');
    if(m_hw_name.size()==0)
    {
      std::cerr << cnr_logger::RED() << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " ;
      std::cerr << "The namespaces are incorrect. " << std::endl;
      std::cerr << " root ns (the namespace of the hw): " << root_nh.getNamespace() << std::endl;
      std::cerr << " controller ns (the namespace of the ctrl): " << controller_nh.getNamespace() << cnr_logger::RST()
                << std::endl;
      return false;
    }
    l = __LINE__;
    if(m_hw_name  .at(0) == '_') m_hw_name  .erase(0, 1);
    l = __LINE__;
    std::replace(m_ctrl_name.begin(), m_ctrl_name.end(), '/', '_');
    if(m_ctrl_name.size()==0)
    {
      std::cerr << cnr_logger::RED() << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " ;
      std::cerr << "The namespace are incorrect. " << std::endl;
      std::cerr << " root ns (the namespace of the hw): " << root_nh.getNamespace() << std::endl;
      std::cerr << " controller ns (the namespace of the ctrl): " << controller_nh.getNamespace() << cnr_logger::RST()
                << std::endl;
      return false;
    }
    l = __LINE__;
    if(m_ctrl_name.at(0) == '_') m_ctrl_name.erase(0, 1);

    l = __LINE__;
    m_logger.reset(new cnr_logger::TraceLogger());
    if(!m_logger->init(m_hw_name + "-" + m_ctrl_name, controller_nh.getNamespace(), false, false))
    {
      if(!m_logger->init(m_hw_name + "-" + m_ctrl_name, root_nh.getNamespace(), false, false))
      {
        std::cerr << cnr_logger::RED() << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " ;
        std::cerr << "The logger cannot be configured:" << std::endl;
        std::cerr << " root ns (the namespace of the hw): " << root_nh.getNamespace() << std::endl;
        std::cerr << " controller ns (the namespace of the ctrl): " << controller_nh.getNamespace() << cnr_logger::RST()
                  << std::endl;
        return false;
      }
    }

    CNR_TRACE_START(m_logger);

    m_root_nh       = root_nh;
    m_controller_nh = controller_nh; // handle to callback and remapping
    m_hw            = hw;

    if(!m_root_nh.getParam("sampling_period", m_sampling_period))
    {
      CNR_RETURN_FALSE(m_logger, "The parameter '" + m_root_nh.getNamespace() + "/sampling_period' is not set. Abort");
    }

    int maximum_missing_cycles = 10;
    if(!m_controller_nh.getParam("watchdog", m_watchdog))
    {
      if(!m_controller_nh.getParam("maximum_missing_cycles", maximum_missing_cycles))
      {
        if(!m_root_nh.getParam("watchdog", m_watchdog))
        {
          if(!m_root_nh.getParam("maximum_missing_cycles", maximum_missing_cycles))
          {
            m_watchdog = maximum_missing_cycles * m_sampling_period;
            CNR_WARN(m_logger, "Neither 'watchdog' and 'maximum_missing_cycles' are in the param server"
                     << " the watchdog is super-imposed to " << std::to_string(maximum_missing_cycles)
                     << " times the sampling period, and it results in " << m_watchdog);
          }
          else
          {
            m_watchdog = maximum_missing_cycles * m_sampling_period;
          }
        }
      }
      else
      {
        m_watchdog = maximum_missing_cycles * m_sampling_period;
      }
    }
    CNR_DEBUG(m_logger, "Watchdog: " << m_watchdog);

    m_controller_nh.setCallbackQueue(&m_controller_nh_callback_queue);
    m_status_history.clear();

    realtime_utilities::DiagnosticsInterface::init( m_hw_name, "Ctrl", m_ctrl_name );
    realtime_utilities::DiagnosticsInterface::addTimeTracker("update", m_sampling_period);
    realtime_utilities::DiagnosticsInterface::addTimeTracker("enterUpdate", m_sampling_period);
    realtime_utilities::DiagnosticsInterface::addTimeTracker("doUpdate", m_sampling_period);
    realtime_utilities::DiagnosticsInterface::addTimeTracker("exitUpdate", m_sampling_period);

    l = __LINE__;
    if(!enterInit())
    {
      CNR_RETURN_FALSE(m_logger);
    }

    l = __LINE__;
    if(!doInit())
    {
      CNR_RETURN_FALSE(m_logger);
    }

    l = __LINE__;
    if(!exitInit())
    {
      CNR_RETURN_FALSE(m_logger);
    }
  }
  catch(std::exception& e)
  {
    std::cerr << cnr_logger::RED() << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " ;
    std::cerr << "Exception at line: "
              << std::to_string(l) << " error: " + std::string(e.what());
    std::cerr << " root ns (the namespace of the hw): " << root_nh.getNamespace() << std::endl;
    std::cerr << " controller ns (the namespace of the ctrl): " << controller_nh.getNamespace() << cnr_logger::RST()
              << std::endl;
  }

  bool ret = dump_state("INITIALIZED");
  if(!ret)
  {
    CNR_ERROR(m_logger, "Failed in dumping the state");
    CNR_RETURN_FALSE(m_logger);
  }
  CNR_RETURN_TRUE(m_logger);
}

template<class T>
void Controller<T>::starting(const ros::Time& time)
{
  CNR_TRACE_START(m_logger);
  if(enterStarting() && doStarting(time) && exitStarting())
  {
    dump_state("RUNNING");
    CNR_RETURN_OK(m_logger, void(), "Starting Ok! Ready to go!");
  }
  else
  {
    CNR_ERROR(m_logger, "The starting of the controller failed. Abort Request to ControllerManager sent.");
    if(controller_interface::Controller<T>::abortRequest(time))
    {
      dump_state("ABORTED");
    }
    else
    {
      dump_state("ERROR");
    }
    CNR_RETURN_NOTOK(m_logger, void());
  }
}

template<class T>
void Controller<T>::update(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(m_logger);
  timeSpanStrakcer("update")->tick();

  timeSpanStrakcer("enterUpdate")->tick();
  bool ok = enterUpdate();
  timeSpanStrakcer("enterUpdate")->tock();

  if(ok)
  {
    m_dt = period.toSec() > 1e-4 ? period : ros::Duration(1e-4);
    timeSpanStrakcer("doUpdate")->tick();
    ok = doUpdate(time, period);
    timeSpanStrakcer("doUpdate")->tock();
    if(ok)
    {
      timeSpanStrakcer("exitUpdate")->tick();
      ok = exitUpdate();
      timeSpanStrakcer("exitUpdate")->tick();
    }
  }

  timeSpanStrakcer("update")->tock();

  if(!ok)
  {
    CNR_ERROR(m_logger, "Error in update, stop request called to stop the controller quietly.");
    if(controller_interface::Controller<T>::stopRequest(time))
    {
      dump_state("STOPPED");
    }
    else
    {
      dump_state("ERROR");
    }
    CNR_RETURN_NOTOK_THROTTLE(m_logger, void(), 10.0);
  }
  CNR_RETURN_OK_THROTTLE_DEFAULT(m_logger, void());
}

template<class T>
void Controller<T>::stopping(const ros::Time& time)
{
  CNR_TRACE_START(m_logger);
  if(enterStopping() && doStopping(time) && exitStopping())
  {
    dump_state("STOPPED");
    CNR_RETURN_OK(m_logger, void());
  }
  else
  {
    if(controller_interface::Controller<T>::abortRequest(time))
    {
      dump_state("ABORTED");
    }
    else
    {
      dump_state("ERROR");
    }
    CNR_RETURN_NOTOK(m_logger, void());
  }
}

template<class T>
void Controller<T>::waiting(const ros::Time& time)
{
  CNR_TRACE_START(m_logger);

  if(enterWaiting() && doWaiting(time) && exitWaiting())
  {
    CNR_RETURN_OK(m_logger, void());
  }
  else
  {
    controller_interface::Controller<T>::abortRequest(time);
    CNR_RETURN_NOTOK(m_logger, void());
  }
}

template<class T>
void Controller<T>::aborting(const ros::Time& time)
{
  CNR_TRACE_START(m_logger);
  CNR_EXIT_EX(m_logger, enterAborting() && doAborting(time)  && exitAborting());
}

/////////////////////////////////////

template<class T>
bool Controller<T>::enterInit()
{
  CNR_TRACE_START(m_logger);
  m_dt = ros::Duration(0);
  CNR_RETURN_TRUE(m_logger);
}

template<class T>
bool Controller<T>::exitInit()
{
  CNR_TRACE_START(m_logger);
  CNR_RETURN_TRUE(m_logger);
}

template<class T>
bool Controller<T>::enterStarting()
{
  CNR_TRACE_START(m_logger);
  CNR_RETURN_TRUE(m_logger);
}

template<class T>
bool Controller<T>::exitStarting()
{

  CNR_TRACE_START(m_logger);
  if(!callAvailable( ))
  {
    CNR_WARN(m_logger, "The callback is still not available...");
  }
  CNR_RETURN_TRUE(m_logger);
}

template<class T>
bool Controller<T>::enterUpdate()
{
  CNR_TRACE_START_THROTTLE_DEFAULT(m_logger);
  if(!callAvailable( ))
  {
    dump_state("CALLBACK_TIMEOUT_ERROR");
    CNR_RETURN_FALSE_THROTTLE(m_logger, 5.0, "Callback Timeout Error");
  }

  std::string status = m_status_history.back();
  if( status != "RUNNING" )
  {
    CNR_RETURN_FALSE_THROTTLE(m_logger, 5.0, "The status is not 'RUNNING' as expected. Abort.");
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(m_logger);
}

template<class T>
bool Controller<T>::exitUpdate()
{
  CNR_TRACE_START_THROTTLE_DEFAULT(m_logger);
  if(m_sub.size() > 0)
  {
    ros::WallTime now = ros::WallTime::now();
    ros::WallTime last_message_time = ros::WallTime::now();
    for(size_t idx=0; idx<m_sub.size(); idx++)
    {
      if(m_sub_time.at(idx) == nullptr)
      {
        CNR_WARN_THROTTLE(m_logger, 5.0, "The topic '" + m_sub.at(idx)->getTopic() + "' seems not yet published..");
      }
      else
      {
        if(m_sub_time_track.at(idx))
        {
          m_sub_time.at(idx)->get(last_message_time);
          ros::WallDuration time_span = (now - last_message_time);
          if(time_span.toSec() > m_watchdog)
          {
            CNR_ERROR_THROTTLE(m_logger, 5.0, "Watchdog on subscribed topic '" + m_sub.at(idx)->getTopic()+ "' " +
                                               std::string("time span: ")+std::to_string(time_span.toSec())+
                                               std::string(" watchdog: ")+std::to_string(m_watchdog));
          }
        }
      }
    }
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(m_logger);
}

template<class T>
bool Controller<T>::enterStopping()
{
  CNR_TRACE_START(m_logger);
  CNR_RETURN_TRUE(m_logger);
}

template<class T>
bool Controller<T>::exitStopping()
{
  CNR_TRACE_START(m_logger);
  bool ret = shutdown("STOPPED");
  if(ret)
  {
    m_controller_nh_callback_queue.disable();
  }
  CNR_RETURN_BOOL(m_logger, ret);
}

template<class T>
bool Controller<T>::enterWaiting()
{
  CNR_TRACE_START(m_logger);
  CNR_RETURN_TRUE(m_logger);
}

template<class T>
bool Controller<T>::exitWaiting()
{
  CNR_TRACE_START(m_logger);
  CNR_RETURN_TRUE(m_logger);
}

template<class T>
bool Controller<T>::enterAborting()
{
  CNR_TRACE_START(m_logger);
  CNR_RETURN_TRUE(m_logger);
}

template<class T>
bool Controller<T>::exitAborting()
{
  CNR_TRACE_START(m_logger);
  bool ret = shutdown("ABORTED");
  if(ret)
  {
    m_controller_nh_callback_queue.disable();
  }
  CNR_RETURN_BOOL(m_logger, ret);
}

template<class T>
bool Controller<T>::dump_state(const std::string& status)
{

  if(m_status_history.size() == 0)
  {
    m_status_history.push_back(status);
    m_controller_nh.setParam(cnr::control::last_status_param(m_hw_name, m_ctrl_name), status);
    m_controller_nh.setParam(cnr::control::status_param(m_hw_name, m_ctrl_name), m_status_history);  }
  else
  {
    if(m_status_history.back() != status)
    {
      m_status_history.push_back(status);
      m_controller_nh.setParam(cnr::control::last_status_param(m_hw_name, m_ctrl_name), status);
      m_controller_nh.setParam(cnr::control::status_param(m_hw_name, m_ctrl_name), m_status_history);
    }
  }

  return true;
}/*

template<class T>
bool Controller<T>::dump_state()
{
  std::string last_status = controller_interface::Controller<T>::isAborted()     ?  "ABORTED"
                          : controller_interface::Controller<T>::isInitialized() ?  "INITIALIZED"
                          : controller_interface::Controller<T>::isRunning()     ?  "RUNNING"
                          : controller_interface::Controller<T>::isWaiting()     ?  "WAITING"
                          : controller_interface::Controller<T>::isStopped()     ?  "STOPPED"
                          : "CONSTRUCTED";
  return dump_state(last_status);
}*/


template<class T>
bool Controller<T>::shutdown(const std::string& state_final)
{
  bool ret = false;
  CNR_TRACE_START(m_logger);
  for(auto & t : m_sub)
  {
    t->shutdown();
  }
  for(auto & t : m_pub)
  {
    t->shutdown();
  }

  for( size_t idx=0; idx<m_sub.size();idx++)
  {
    m_sub.at(idx).reset();
    m_sub_time.at(idx).reset();
    m_sub_notifier.at(idx).reset();
  }
  m_sub.clear();
  m_sub_time.clear();
  m_sub_time_track.clear();
  m_sub_notifier.clear();

  for( size_t idx=0; idx<m_pub.size();idx++)
  {
    m_pub.at(idx).reset();
    delete m_pub_start.at(idx);
    delete m_pub_last.at(idx);
  }
  m_pub.clear();
  m_pub_start.clear();
  m_pub_last.clear();
  m_pub_time_track.clear();

  ret = dump_state(state_final != "" ? state_final : "STOPPED" );

  CNR_RETURN_BOOL(m_logger, ret);
}


template<typename T> template<typename M>
size_t Controller<T>::add_publisher(const std::string &topic, uint32_t queue_size, bool latch, bool enable_watchdog)
{
  m_pub.push_back(std::shared_ptr<ros::Publisher>(
      new ros::Publisher(m_controller_nh.advertise< M >(topic, queue_size, latch))) );
  m_pub_start.push_back(nullptr);
  m_pub_last.push_back(nullptr);
  m_pub_time_track.push_back(enable_watchdog);
  return m_pub.size()-1;
}

template<typename T> template<typename M>
bool Controller<T>::publish(const size_t& idx, const M &message)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->m_logger);
  if(idx >=m_pub.size())
  {
    CNR_RETURN_FALSE(this->m_logger,
          "The index is out of range (idx:" +std::to_string(idx) + " size: " + std::to_string(m_pub.size()) +")" );
  }

  if(!m_pub.at(idx))
  {
    CNR_RETURN_FALSE(this->m_logger, "The publisher is bad configured, it is a nullptr?!");
  }

  m_pub.at(idx)->publish(message);

  if(m_pub_time_track.at(idx))
  {
    auto n = std::chrono::high_resolution_clock::now();
    if(m_pub_start.at(idx) == nullptr)
    {
      m_pub_start.at(idx) = new std::chrono::high_resolution_clock::time_point();
      *m_pub_start.at(idx) = n;
      m_pub_last.at(idx) = new std::chrono::high_resolution_clock::time_point();
      *m_pub_last.at(idx) = n;
    }
    std::chrono::duration<double> time_span = (n -  *m_pub_last.at(idx));
    *m_pub_last.at(idx)  = n;
    if(time_span.count() > m_watchdog)
    {
      CNR_RETURN_FALSE(this->m_logger, "The publisher has not been called within the foreseen watchdog."
                +std::string("Time span: ") + std::to_string(time_span.count()) + ", "
                +std::string("watchdog: " ) + std::to_string(m_watchdog));
    }
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->m_logger);
}

template<typename T> template<typename M>
size_t Controller<T>::add_subscriber(const std::string &topic,
                                     uint32_t queue_size,
                                     boost::function<void(const boost::shared_ptr<M const>& msg)> callback,
                                     bool enable_watchdog)
{
  std::shared_ptr<ros_helper::SubscriptionNotifier<M> > sub(
        new ros_helper::SubscriptionNotifier<M>(m_controller_nh, topic, queue_size, callback) );

  m_sub_notifier.push_back( sub );                    // it's needed to destroy memeory only when ~Controller is called.
                                                      // the trick of using std::shared_ptr<void> allows to do not care
                                                      // with the template type of the object
  m_sub.push_back(sub->getSubscriber());
  m_sub_time.push_back(sub->getMsgReceivedTime());
  m_sub_time_track.push_back(enable_watchdog);

  return m_sub.size()-1;
}

template<class T>
std::shared_ptr<ros::Subscriber> Controller<T>::getSubscriber(const size_t& idx)
{
  if(idx<m_sub.size())
    return m_sub.at(idx);
  return nullptr;
}

template<class T>
std::shared_ptr<ros::Publisher> Controller<T>::getPublisher(const size_t& idx)
{
  if(idx<m_pub.size())
    return m_pub.at(idx);
  return nullptr;
}

template<class T>
bool Controller<T>::callAvailable( )
{
  m_controller_nh_callback_queue.callAvailable();
  return true;
}



}  // namespace control
}  // namespace cnr

#endif  // CNR_CONTROLLER_INTFERFACE_CNR_CONTROLLER_INTFERFACE_IMPL_H
