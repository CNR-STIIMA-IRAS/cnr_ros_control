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

#ifndef CNR_CONTROLLER_INTERFACE_CNR_CONTROLLER_INTERFACE_H
#define CNR_CONTROLLER_INTERFACE_CNR_CONTROLLER_INTERFACE_H

#include <ctime>
#include <chrono>
#include <algorithm>
#include <mutex>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_interface/controller.h>
#include <controller_manager_msgs/ControllerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_controller_interface/internal/utils.h>
#include <realtime_utilities/time_span_tracker.h>
#include <subscription_notifier/subscription_notifier.h>

namespace cnr_controller_interface
{

//============ FUNCTIONS TO DEFINE THE PARAMETERS WHERE THE CTRL STATUS IS LOADED
std::vector<std::string> get_names(const std::vector< controller_manager_msgs::ControllerState >& controllers);
std::string              ctrl_list_param(const std::string& hw_name);
std::string              last_status_param(const std::string& hw_name,
                                           const std::string& ctrl_name);
std::string              status_param(const std::string& hw_name,
                                      const std::string& ctrl_name);
bool                     get_state(const std::string& hw_name,
                                   const std::string& ctrl_name,
                                   std::string& status,
                                   std::string& error,
                                   const ros::Duration& watchdog = ros::Duration(0.0));

template<typename T>
boost::shared_ptr<T> to_boost_shared_ptr(std::shared_ptr<T>& ptr)
{
    return boost::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
}

template<typename T>
std::shared_ptr<T> to_std_shared_ptr(boost::shared_ptr<T>& ptr)
{
    return std::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
}

class ControllerDiagnostic
{
public:

  /**
   * @brief add_diagnostic_message
   * @param msg
   * @param name
   * @param level
   * @param verbose
   */
  virtual void add_diagnostic_message(const std::string& msg,
                                      const std::string& name,
                                      const std::string& level,
                                      const bool&        verbose) {}

  virtual void diagnostics     (diagnostic_updater::DiagnosticStatusWrapper &stat, int level);
  virtual void diagnosticsInfo (diagnostic_updater::DiagnosticStatusWrapper &stat)           ;
  virtual void diagnosticsWarn (diagnostic_updater::DiagnosticStatusWrapper &stat)           ;
  virtual void diagnosticsError(diagnostic_updater::DiagnosticStatusWrapper &stat)           ;
  virtual void diagnosticsPerformance(diagnostic_updater::DiagnosticStatusWrapper &stat)     ;

protected:

  std::string                                           m_hw_name;
  std::string                                           m_ctrl_name;
  mutable std::mutex                                    m_mutex;
  mutable diagnostic_msgs::DiagnosticArray              m_diagnostic;
  std::shared_ptr<cnr_logger::TraceLogger>              m_logger;
  std::vector<std::string>                              m_status_history;
  std::shared_ptr<realtime_utilities::TimeSpanTracker>  m_time_span_tracker;
  double                                                m_sampling_period;
  double                                                m_watchdog;


};

template< class T >
class Controller: public ::controller_interface::Controller< T >, public ControllerDiagnostic
{
public:

  ~Controller();

  bool init(T*, ros::NodeHandle&)                                            final
  {
    return true;
  }
  bool init(T* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) final;
  void starting(const ros::Time& time)                                       final;
  void update(const ros::Time& time, const ros::Duration& period)            final;
  void stopping(const ros::Time& time)                                       final;
  void waiting(const ros::Time& time)                                        final;
  void aborting(const ros::Time& time)                                       final;

  std::shared_ptr<cnr_logger::TraceLogger>& logger() { return m_logger; }

public:
  virtual bool doInit()
  {
    return true;
  }
  virtual bool doStarting(const ros::Time& /*time*/)
  {
    return true;
  }
  virtual bool doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
  {
    return true;
  }
  virtual bool doStopping(const ros::Time& /*time*/)
  {
    return true;
  }
  virtual bool doWaiting(const ros::Time& /*time*/)
  {
    return true;
  }
  virtual bool doAborting(const ros::Time& /*time*/)
  {
    return true;
  }
  std::string getRootNamespace()
  {
    return m_root_nh.getNamespace();
  }
  std::string getControllerNamespace()
  {
    return m_controller_nh.getNamespace();
  }
  ros::NodeHandle& getRootNh()
  {
    return m_root_nh;
  }
  ros::NodeHandle& getControllerNh()
  {
    return m_controller_nh;
  }

  /**
   * @brief shutdown
   * @param state_final
   * @return
   */
  bool shutdown(const std::string& state_final);
  
  /**
   * @brief add_publisher
   * @param id
   * @param topic
   * @param queue_size
   * @param latch
   */
  template<typename M> void add_publisher(const std::string& id,
                                          const std::string &topic,
                                          uint32_t queue_size,
                                          bool latch = false);

  /**
   * @brief publish
   * @param id
   * @param message
   * @return
   */
  template<typename M>
  bool publish(const std::string& id, const M &message);

  /**
   * @brief add_subscriber
   * @param id
   * @param topic
   * @param queue_size
   * @param obj
   * @param transport_hints
   */
  template<typename M>
  void add_subscriber(const std::string& id,
                      const std::string &topic,
                      uint32_t queue_size,
                      boost::function<void(const boost::shared_ptr<M const>& msg)> callback,
                      bool enable_watchdog = true);

  std::shared_ptr<ros::Subscriber> getSubscriber(const std::string& id);
  std::shared_ptr<ros::Publisher>  getPublisher(const std::string& id);

  void add_diagnostic_message(const std::string& msg,
                              const std::string& name,
                              const std::string& level,
                              const bool&        verbose) override;

protected:

  virtual bool enterInit();
  virtual bool exitInit();

  virtual bool enterStarting();
  virtual bool exitStarting();

  virtual bool enterUpdate();
  virtual bool exitUpdate();

  // stopping
  virtual bool enterStopping();
  virtual bool exitStopping();

  // waiting
  virtual bool enterWaiting();
  virtual bool exitWaiting();

  // aborting
  virtual bool enterAborting();
  virtual bool exitAborting();

  bool dump_state(const std::string& status);
  bool dump_state();

protected:
  T*            m_hw;
  ros::Duration m_dt;

private:
  ros::NodeHandle     m_root_nh;
  ros::NodeHandle     m_controller_nh;
  ros::CallbackQueue  m_controller_nh_callback_queue;

  struct Publisher
  {
    std::shared_ptr<ros::Publisher>                 pub;
    std::chrono::high_resolution_clock::time_point* start = nullptr;
    std::chrono::high_resolution_clock::time_point  last;
    std::chrono::duration<double>                   time_span;
  };


  std::map<std::string, Publisher > m_pub;
  std::map<std::string, std::shared_ptr<ros::Subscriber> > m_sub;
  std::map<std::string, ros_helper::WallTimeMTPtr > m_sub_time;

  bool callAvailable( );

};

}  // namespace cnr_controller_interface


#include <cnr_controller_interface/cnr_controller_interface_impl.h>

#endif  // CNR_CONTROLLER_INTERFACE_CNR_CONTROLLER_INTERFACE_H
