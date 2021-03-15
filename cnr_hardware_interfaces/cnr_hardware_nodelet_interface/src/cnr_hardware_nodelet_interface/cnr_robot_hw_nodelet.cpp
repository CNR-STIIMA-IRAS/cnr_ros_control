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

#if PREEMPTIVE_RT == 1
  #include <cinttypes>
  #include <csignal>
  #define USE_TIMER_REALTIME_UTILS  1
  #define PRE_ALLOCATION_SIZE       1024*1024*1024
  #define RT_STACK_SIZE             1024*1024
#else
  //#define USE_TIMERFD
  #define USE_WALLRATE
#endif

#include <cstring>
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <pluginlib/class_list_macros.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <realtime_utilities/realtime_utilities.h>
#include <cnr_controller_interface_params/cnr_controller_interface_params.h>
#include <cnr_hardware_interface/cnr_robot_hw.h>
#include <configuration_msgs/SendMessage.h>

#include <cnr_hardware_nodelet_interface/cnr_robot_hw_nodelet.h>

PLUGINLIB_EXPORT_CLASS(cnr_hardware_nodelet_interface::RobotHwNodelet, nodelet::Nodelet)


#if defined(USE_TIMERFD)

#include <sys/time.h>
#include <sys/timerfd.h>

struct periodic_info {
  int timer_fd;
  unsigned long long wakeups_missed;
};

static int make_periodic(unsigned int period, struct periodic_info *info)
{
  int ret;
  unsigned int ns;
  unsigned int sec;
  int fd;
  struct itimerspec itval;

  /* Create the timer */
  fd = timerfd_create(CLOCK_MONOTONIC, 0);
  info->wakeups_missed = 0;
  info->timer_fd = fd;
  if (fd == -1)
    return fd;

  /* Make the timer periodic */
  sec = period / 1000000;
  ns = (period - (sec * 1000000)) * 1000;
  itval.it_interval.tv_sec = sec;
  itval.it_interval.tv_nsec = ns;
  itval.it_value.tv_sec = sec;
  itval.it_value.tv_nsec = ns;
  ret = timerfd_settime(fd, 0, &itval, NULL);
  return ret;
}

static void wait_period(struct periodic_info *info)
{
  unsigned long long missed;
  int ret;

  /* Wait for the next timer event. If we have missed any the
     number is written to "missed" */
  ret = read(info->timer_fd, &missed, sizeof(missed));
  if (ret == -1) {
    perror("read timer");
    return;
  }

  info->wakeups_missed += missed;
}
#endif

namespace cnr_hardware_nodelet_interface
{

inline std::string extractRobotName(const std::string& hw_namespace)
{
  std::string hw_name = hw_namespace;
  hw_name      .erase(0, 1);
  std::replace(hw_name.begin(), hw_name.end(), '/', '_');
  return hw_name;
}



template <typename T>
std::string to_string(const T a_value, const int n = 5)
{
  std::ostringstream out;
  out.precision(n);
  out << std::fixed << a_value;
  return out.str();
}


RobotHwNodelet::~RobotHwNodelet()
{
  CNR_TRACE_START(m_logger, "Shutting Down the nodelet...");

  try
  {
    CNR_INFO(m_logger, "Reset the HW (watchdog: 10sec)");
    std::vector<std::string> ctrls;
    if (ros::param::has(cnr::control::ctrl_list_param(m_hw_name)))
    {
      ros::param::get(cnr::control::ctrl_list_param(m_hw_name), ctrls);
      for (const auto ctrl : ctrls)
      {
        std::string last_status;
        ros::param::get(cnr::control::last_status_param(m_hw_name, ctrl), last_status);
        if (last_status != "UNLOADED")
        {
          CNR_INFO(m_logger, "Last Status Tracked: " << last_status);
          if (!m_cmp->unloadController(ctrl, ros::Duration(1.0)))
          {
            CNR_INFO(m_logger, "Error in unloading the controller " << ctrl);
          }
        }
      }
    }

    CNR_WARN(m_logger, "Join the diagnostic thread");
    m_stop_diagnostic_thread = true;
    if (m_diagnostics_thread.joinable())
    {
      m_diagnostics_thread.join();
    }

    CNR_WARN(m_logger, "Join the update thread");
    m_stop_update_thread = true;
    if (m_update_thread.joinable())
    {
      m_update_thread.join();
    }

    m_hw.reset();
  }
  catch (std::exception& e)
  {
    CNR_FATAL(m_logger, "Error in shutting down the RobotHw: " << e.what());
  }
  catch (...)
  {
    CNR_FATAL(m_logger, "Error in shuttind down the RobotHw: ");
  }
  CNR_TRACE(m_logger, "[ DONE] Shutted Down");
}

void RobotHwNodelet::onInit()
{
  m_hw_namespace = getPrivateNodeHandle().getNamespace();
  m_hw_name      = extractRobotName(m_hw_namespace);
  //m_updater.setHardwareID(m_hw_name);
  m_logger.reset(new cnr_logger::TraceLogger("NL_" + m_hw_name, m_hw_namespace));

  try
  {
    CNR_TRACE_START(m_logger);
    if(enterOnInit() && doOnInit() && exitOnInit())
    {
      m_stop_update_thread = m_stop_diagnostic_thread = false;
      CNR_RETURN_OK(m_logger, void());
    }
  }
  catch (std::exception& e)
  {
    dump_state(cnr_hardware_interface::ERROR);
    CNR_RETURN_NOTOK(m_logger, void(), m_hw_name + ": ExitOnInit failed. Exception caught: " + std::string(e.what()));
  }
  catch (...)
  {
    dump_state(cnr_hardware_interface::ERROR);
    CNR_RETURN_NOTOK(m_logger, void(), m_hw_name + ": ExitOnInit failed. UNhandled Exception");
  }
}

bool RobotHwNodelet::doOnInit()
{
  CNR_TRACE_START(m_logger);
  std::string robot_type;
  try
  {
    if (!m_hw_nh.getParam("type", robot_type))
    {
      CNR_FATAL(m_logger, "The param '" << m_hw_nh.getNamespace() << "/type' is missing! Abort.");
      CNR_RETURN_FALSE(m_logger);
    }

    CNR_DEBUG(m_logger, "Loading instance: '"  << robot_type << "'");
    CNR_DEBUG(m_logger, "Is Class Available? " << (m_robot_hw_plugin_loader->isClassAvailable(robot_type) ? "YES" : "NO"));
    CNR_DEBUG(m_logger, "Name of the class ? " <<  m_robot_hw_plugin_loader->getName(robot_type));

    m_robot_hw_plugin_loader->loadLibraryForClass(robot_type);
    auto hw = m_robot_hw_plugin_loader->createInstance(robot_type);
    m_hw = cnr_hardware_interface::to_std_ptr( hw );
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = m_robot_hw_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    CNR_ERROR(m_logger, "Exception while loading '" << robot_type << "': " << ex.what() << "\n Available plugins: " << ss.str());
  }
  catch (std::exception& e)
  {
    CNR_RETURN_FALSE(m_logger, "OnInit of FakeRobotHw failed! what: " + std::string(e.what()));
  }
  CNR_RETURN_TRUE(m_logger);
}

bool RobotHwNodelet::enterOnInit()
{
  CNR_TRACE_START(m_logger);
  m_stop_update_thread = m_stop_diagnostic_thread = false;

  // getNodeHandle(), parent nodelet handle (typically "/")  --- getPrivateNodeHandle() -> /nodelet_name/
  m_nh     = getNodeHandle();
  m_hw_nh  = getPrivateNodeHandle();

  double sampling_period = 0.001;
  if (!m_hw_nh.getParam("sampling_period", sampling_period))
  {
    CNR_WARN(m_logger, m_hw_namespace + "/sampling_period' does not exist, set equal to 0.001");
    sampling_period = 1.0e-3;
  }

  m_period              = ros::Duration(sampling_period);

  realtime_utilities::DiagnosticsInterface::init(m_hw_name, "RobotHwNodelet", "Main Loop");
  realtime_utilities::DiagnosticsInterface::addTimeTracker("cycle",sampling_period);
  realtime_utilities::DiagnosticsInterface::addTimeTracker("read",sampling_period);
  realtime_utilities::DiagnosticsInterface::addTimeTracker("write",sampling_period);
  realtime_utilities::DiagnosticsInterface::addTimeTracker("update",sampling_period);
  
  m_robot_hw_plugin_loader.reset(
    new pluginlib::ClassLoader<hardware_interface::RobotHW>("hardware_interface", "hardware_interface::RobotHW"));

  boost::function<bool(configuration_msgs::SendMessage::Request &req,configuration_msgs::SendMessage::Response &res)>
      callback =
      [this] (configuration_msgs::SendMessage::Request &req,configuration_msgs::SendMessage::Response &res)
      {
        CNR_TRACE(this->m_logger, req.message.data);
        return true;
      };
  m_mail_service = getPrivateNodeHandle().advertiseService("mail", callback);

  CNR_RETURN_TRUE(m_logger);
}

bool RobotHwNodelet::exitOnInit()
{
  CNR_TRACE_START(m_logger);
  CNR_DEBUG(m_logger, "Triggering the start of the main thread");

  if (m_hw == nullptr)
  {
    CNR_RETURN_FALSE(m_logger, "The RobotHw has not been properly initialized in the doOnInit() function. Abort.");
  }

  if (!m_hw->init(m_nh, m_hw_nh))
  {
    CNR_RETURN_FALSE(m_logger, "The RobotHw '" + m_hw_name + "'Initialization failed. Abort.");
  }

  if (m_hw->getRobotHwNamespace() != m_hw_namespace)
  {
    CNR_WARN(m_logger, "Mismatch between the namespace of the RobotHw '" + m_hw->getRobotHwNamespace()
             + "'and of the hardware label from configuration manager '" + m_hw_namespace + "'. Is it right?");
  }

  m_cmp.reset(new cnr_controller_manager_interface::ControllerManagerProxy( m_logger, m_hw_name, m_hw.get(), m_hw_nh));

  m_update_thread_state = ON_INIT;
  ros::Time start       = ros::Time::now();
  m_update_thread       = std::thread(&cnr_hardware_nodelet_interface::RobotHwNodelet::controlUpdateThread, this);
  while(m_update_thread_state != RUNNING)
  {
    if (m_update_thread_state == ON_ERROR)
    {
      CNR_RETURN_FALSE(m_logger, "The update thread is in Error. Abort.");
    }
    if (m_update_thread_state == EXPIRED)
    {
      CNR_RETURN_FALSE(m_logger, "The update thread is expired. Something went wrong during the start. Abort.");
    }
    if ((ros::Time::now() - start).toSec() > 10.0)
    {
      CNR_RETURN_FALSE(m_logger, "Timeout Expired. The update thread did not yet started. Abort.");
    }
    ros::Duration(0.05).sleep();
  }

  CNR_DEBUG(m_logger, "Diagnostic started ");
  CNR_DEBUG(m_logger, "Frequency: '" << 1.0 / m_period.toSec() << "'");
  CNR_DEBUG(m_logger, "Supported Hardware Interfaces:");
  for (auto& elem : m_hw->getNames())
  {
    CNR_DEBUG(m_logger, "[ " << m_hw_name << " ] - '" <<  elem << "'");
  }

  m_diagnostics_thread_state = ON_INIT;
  start                      = ros::Time::now();
  m_diagnostics_thread       = std::thread(&cnr_hardware_nodelet_interface::RobotHwNodelet::diagnosticsThread, this);
  while(m_diagnostics_thread_state != RUNNING)
  {
    if (m_diagnostics_thread_state == ON_ERROR)
    {
      CNR_RETURN_FALSE(m_logger, "Main thread in ERROR");
    }
    if ((ros::Time::now() - start).toSec() > 10.0)
    {
      CNR_RETURN_FALSE(m_logger, "Timeout Exipred. Main Thread did not started yet. Abort.");
    }
    ros::Duration(0.05).sleep();
  }
  CNR_DEBUG(m_logger, "The main is running");
  CNR_RETURN_TRUE(m_logger);
}

void RobotHwNodelet::diagnosticsThread()
{
  CNR_INFO(m_logger, "Diagnostics Thread Started");
  diagnostic_updater::Updater   updater(m_hw_nh, ros::NodeHandle("~"), "/" + m_hw_name);

  updater.setHardwareID(m_hw_name);
  std::string id = "RobotHW | ";

  realtime_utilities::DiagnosticsInterfacePtr hw_d = std::dynamic_pointer_cast<realtime_utilities::DiagnosticsInterface>(m_hw);
  realtime_utilities::DiagnosticsInterface*   hwn_d = dynamic_cast<realtime_utilities::DiagnosticsInterface*>(this);

  updater.add(id + "Info"      , hw_d.get(), &cnr_hardware_interface::RobotHW::diagnosticsInfo);
  updater.add(id + "Warning"   , hw_d.get(), &cnr_hardware_interface::RobotHW::diagnosticsWarn);
  updater.add(id + "Error"     , hw_d.get(), &cnr_hardware_interface::RobotHW::diagnosticsError);
  updater.add(id + "Timers"    , hw_d.get(), &cnr_hardware_interface::RobotHW::diagnosticsPerformance);
  updater.add(id + "Main Loop (nodelet)", hwn_d, &RobotHwNodelet::diagnosticsPerformance);

  id = "Ctrl | ";
  updater.add(id + "Info"    , m_cmp.get(), &cnr_controller_manager_interface::ControllerManagerProxy::diagnosticsInfo);
  updater.add(id + "Warning" , m_cmp.get(), &cnr_controller_manager_interface::ControllerManagerProxy::diagnosticsWarn);
  updater.add(id + "Error"   , m_cmp.get(), &cnr_controller_manager_interface::ControllerManagerProxy::diagnosticsError);
  updater.add(id + "Timers"  , m_cmp.get(), &cnr_controller_manager_interface::ControllerManagerProxy::diagnosticsPerformance);

  ros::WallDuration wd(updater.getPeriod());
  try
  {
    m_diagnostics_thread_state = RUNNING;
    while (ros::ok() && !m_stop_diagnostic_thread)
    {
      updater.update();
      wd.sleep();

      // TO DO: ADD diagnosticsError HWNodelet
      bool hardware_interface_with_error = (m_hw == nullptr) || (m_hw->getStatus() == cnr_hardware_interface::ERROR);
      if(hardware_interface_with_error)
      {
        CNR_FATAL_THROTTLE(m_logger, 10, "The Hardware interface '" << m_hw_name << "' is in error state, shutting down");
      }
      if (m_update_thread_state == ON_ERROR)
      {
        CNR_FATAL(m_logger, "The update thread is in Error. Abort.");
      }
      if (m_update_thread_state == EXPIRED)
      {
        CNR_FATAL(m_logger, "The update thread is expired. Something went wrong during the start. Abort.");
      }
    }
    CNR_DEBUG(m_logger, "Diagnostic finished.");
  }
  catch (std::exception& e)
  {
    m_stop_diagnostic_thread = true;
    CNR_FATAL(m_logger, "Hardware interface is in error state. Exception: " <<  e.what());
    m_diagnostics_thread_state = ON_ERROR;
  }
  m_diagnostics_thread_state = EXPIRED;
  CNR_WARN(m_logger, "Diagnositcs Thread Expired");
  return;
}


void RobotHwNodelet::controlUpdateThread()
{
  CNR_WARN(m_logger, "Start update thread (period: " << m_period << ")");

#if PREEMPTIVE_RT
  realtime_utilities::period_info  pinfo;
  if(!realtime_utilities::rt_init_thread(RT_STACK_SIZE, sched_get_priority_max(SCHED_RR)-2, SCHED_RR, &pinfo, m_period.toNSec()) )
  {
    CNR_ERROR(m_logger, "Failed in setting thread rt properties. Exit. ");
    std::raise(SIGINT);
    return;
  }
#endif

  m_update_thread_state = RUNNING;
#if defined(USE_TIMER_REALTIME_UTILS)
  realtime_utilities::period_info ptarget;
  ptarget.period_ns = m_period.toNSec();

  clock_gettime(CLOCK_MONOTONIC, &(ptarget.next_period));
  ptarget.next_period.tv_nsec  =  ( (ptarget.next_period.tv_nsec / 1000000) + 1 ) * 1000000 + 5e7;  // approx to ms
  ptarget.next_period.tv_sec  += 0.0;
#elif defined(USE_TIMERFD)
  struct periodic_info info;
  make_periodic( (m_period.toNSec() / 1000 ), &info);
#elif defined(USE_WALLRATE)
  ros::WallRate wr( m_period );
#endif

  try
  {
    if(!m_hw->initRT())
    {
      CNR_ERROR_THROTTLE(m_logger, 5.0, "Error in RT init. Abort");
      m_stop_update_thread = true;
      m_update_thread_state = EXPIRED;
      return;
    }
  }
  catch (std::exception& e)
  {
    CNR_ERROR(m_logger, "updateThread error call hardware interface initRT(): " << e.what());
    m_stop_update_thread = true;
    m_update_thread_state = EXPIRED;
    return;
  }


  while (ros::ok())
  {
#if defined(USE_TIMER_REALTIME_UTILS)
  realtime_utilities::timer_wait_rest_of_period(&(ptarget.next_period));
  realtime_utilities::timer_inc_period(&ptarget);
#elif defined(USE_TIMERFD)
  wait_period(&info);
#elif defined(USE_WALLRATE)
  wr.sleep();
#endif

    timeSpanStrakcer("cycle")->time_span();
    if (m_stop_update_thread)
    {
      CNR_WARN(m_logger, "Exiting update thread of the hardware interface because a stop has been triggered.");
      break;
    }

    try
    {
      timeSpanStrakcer("read")->tick();
      m_hw->read(ros::Time::now(), m_period);
      timeSpanStrakcer("read")->tock();
    }
    catch (std::exception& e)
    {
      CNR_ERROR(m_logger, "updateThread error call hardware interface read(): " << e.what());
      break;
    }

    try
    {
      timeSpanStrakcer("update")->tick();
      m_cmp->update(ros::Time::now(), m_period);
      timeSpanStrakcer("update")->tock();
    }
    catch (std::exception& e)
    {
      CNR_WARN(m_logger, "updateThread error call controller manager update(): " << std::string(e.what()));
      m_stop_update_thread = true;
      m_update_thread_state = ON_ERROR;
      return;
    }
    try
    {
      timeSpanStrakcer("write")->tick();
      m_hw->write(ros::Time::now(), m_period);
      timeSpanStrakcer("write")->tock();
    }
    catch (std::exception& e)
    {
      CNR_WARN(m_logger, "updateThread error call hardware interface write() " << e.what());
    }

    if (m_hw->getStatus() == cnr_hardware_interface::ERROR)
    {
      CNR_ERROR_THROTTLE(m_logger, 1.0, "RobotHw is in error");
      dump_state(cnr_hardware_interface::ERROR);
    }
  }

  m_stop_update_thread = true;
  m_update_thread_state = EXPIRED;
  CNR_WARN(m_logger, "EXIT UPDATE THREAD");
}

bool RobotHwNodelet::dump_state(const cnr_hardware_interface::StatusHw& status) const
{
  ros::param::set(cnr_hardware_interface::last_status_param(m_hw_namespace), status);
  return true;
}

}
