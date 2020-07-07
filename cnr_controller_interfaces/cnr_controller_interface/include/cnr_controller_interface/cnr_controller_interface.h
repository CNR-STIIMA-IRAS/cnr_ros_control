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
/**
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

#ifndef __CNR__CONTROLLER_INTERFACE__
#define __CNR__CONTROLLER_INTERFACE__

#include <ctime>
#include <chrono>
#include <algorithm>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_interface/controller.h>
#include <controller_manager_msgs/ControllerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <cnr_logger/cnr_logger.h>
#include <cnr_controller_interface/internal/utils.h>

namespace cnr_controller_interface
{


//============ FUNCTIONS TO DEFINE THE PARAMETERS WHERE THE CTRL STATUS IS LOADED
std::vector<std::string> get_names(const std::vector< controller_manager_msgs::ControllerState >& controllers);
std::string              ctrl_list_param(const std::string& hw_name);
std::string              last_status_param(const std::string& hw_name, const std::string& ctrl_name);
std::string              status_param(const std::string& hw_name, const std::string& ctrl_name);
bool                     get_state(const std::string& hw_name, const std::string& ctrl_name, std::string& status, std::string& error, const ros::Duration& watchdog = ros::Duration(0.0));

template< class T >
class Controller: public ::controller_interface::Controller< T >
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

  void add_diagnostic_message(const std::string& msg, const std::string& name, const std::string& level, const bool verbose);

  bool shutdown(const std::string& state_final)
  {
    m_controller_nh_callback_queue.callAvailable(ros::WallDuration(5.0));
    for (auto & t : m_sub)
    {
      t.second.sub.shutdown();
    }
    for (auto & t : m_pub)
    {
      t.second.pub.shutdown();
    }
    if(state_final=="")
    {
      return dump_state();
    }
    return dump_state(state_final);
  }
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


  template<class M>
  void add_publisher(const std::string& id, const std::string &topic, uint32_t queue_size, bool latch = false);

  template<typename M>
  bool publish(const std::string& id, const M &message);

  template<class M, class K>
  void add_subscriber(const std::string& id, const std::string &topic, uint32_t queue_size, void(K::*fp)(M), K *obj, const ros::TransportHints &transport_hints = ros::TransportHints());

  bool tick(const std::string& id);

  ros::Subscriber& getSubscriber(const std::string& id);
  ros::Publisher&  getPublisher(const std::string& id);

protected:

  T*                                       m_hw;
  std::string                              m_hw_name;
  std::string                              m_ctrl_name;
  std::shared_ptr<cnr_logger::TraceLogger> m_logger;
  diagnostic_msgs::DiagnosticArray         m_diagnostic;
  ros::Publisher                           m_diagnostics_pub;

  std::vector<std::string>                 m_status_history;

  double                                   m_watchdog;
  double                                   m_sampling_period;

private:
  ros::NodeHandle                          m_root_nh;
  ros::NodeHandle                          m_controller_nh;
  ros::CallbackQueue                       m_controller_nh_callback_queue;

  struct Publisher
  {
    ros::Publisher pub;
    std::chrono::high_resolution_clock::time_point* start = nullptr;
    std::chrono::high_resolution_clock::time_point  last;
    std::chrono::duration<double>                   time_span;
  };

  struct Subscriber
  {
    ros::Subscriber sub;
    std::chrono::high_resolution_clock::time_point* start = nullptr;
    std::chrono::high_resolution_clock::time_point  last;
    std::chrono::duration<double>                   time_span;
  };

  std::map< std::string, Publisher  > m_pub;
  std::map<std::string,  Subscriber > m_sub;

  bool callAvailable(const double watchdog);

};

} // cnr_controller_interface


#include <cnr_controller_interface/cnr_controller_interface_impl.h>

#endif
