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
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSf
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
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_interface/controller.h>

#include <cnr_logger/cnr_logger.h>
#include <realtime_utilities/diagnostics_interface.h>
#include <subscription_notifier/subscription_notifier.h> //ros_helper::WallTimeMTPr
namespace cnr
{
namespace control
{


template<class T>
class Controller: public controller_interface::Controller<T>,
                  public realtime_utilities::DiagnosticsInterface
{
public:

  virtual ~Controller();

  bool init(T* hw, ros::NodeHandle&)                                         final
  {
    return hw;
  }
  bool init(T* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) final;
  void starting(const ros::Time& time)                                       final;
  void update(const ros::Time& time, const ros::Duration& period)            final;
  void stopping(const ros::Time& time)                                       final;
  void waiting(const ros::Time& time)                                        final;
  void aborting(const ros::Time& time)                                       final;

  cnr_logger::TraceLoggerPtr logger() { return m_logger; }

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
  template<typename M>
  size_t add_publisher(const std::string &topic,
                       uint32_t queue_size,
                       bool latch = false,
                       bool enable_watchdog = false);

  /**
   * @brief publish
   * @param id
   * @param message
   * @return
   */
  template<typename M>
  bool publish(const size_t& idx, const M &message);

  /**
   * @brief publish
   * @param id
   * @param message
   * @return
   */
  template<typename M>
  bool publish(const size_t& idx, const boost::shared_ptr<M>& message);

  /**
   * @brief Allows the user to easily add a subscriber, eith the correct callback queue, 
   *        and the timing check
   * 
   * The usage of the function is
   * class A : public Controller<T>
   * {
   *    bool doInit()
   *    {
   *       this->template add_subscriber< type of the message>(topic, queue_size, callbac, watchdog)
   *    }
   * };
   * @param id
   * @param topic
   * @param queue_size
   * @param obj
   * @param transport_hints
   */
  template<typename M>
  size_t add_subscriber(const std::string &topic,
                        uint32_t queue_size,
                        boost::function<void(const boost::shared_ptr<M const>& msg)> callback,
                        bool enable_watchdog = true);

  std::shared_ptr<ros::Subscriber> getSubscriber(const size_t& id);
  std::shared_ptr<ros::Publisher>  getPublisher(const size_t &id);

//protected:

  virtual bool prepareInit(T* hw, const std::string& hw_name, const std::string& ctrl_name, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
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
  //bool dump_state();

protected:
  T*            m_hw;
  ros::Duration m_dt;
  cnr_logger::TraceLoggerPtr  m_logger;
  std::string                 m_hw_name;
  std::string                 m_ctrl_name;
  double                      m_sampling_period;
  double                      m_watchdog;
  std::vector<std::string>    m_status_history;

private:
  ros::NodeHandle     m_root_nh;
  ros::NodeHandle     m_controller_nh;
  ros::CallbackQueue  m_controller_nh_callback_queue;

  std::vector<std::shared_ptr<ros::Publisher>>                 m_pub;
  std::vector<std::chrono::high_resolution_clock::time_point*> m_pub_start;
  std::vector<std::chrono::high_resolution_clock::time_point*> m_pub_last;
  std::vector<bool>                                            m_pub_time_track;

  std::vector<std::shared_ptr<void>>            m_sub_notifier;
  std::vector<std::shared_ptr<ros::Subscriber>> m_sub;
  std::vector<ros_helper::WallTimeMTPtr>        m_sub_time;
  std::vector<bool>                             m_sub_time_track;

  bool callAvailable( );
};

}   // namespace cnr
}   // namespace control

#include <cnr_controller_interface/internal/cnr_controller_interface_impl.h>

#endif  // CNR_CONTROLLER_INTERFACE_CNR_CONTROLLER_INTERFACE_H
