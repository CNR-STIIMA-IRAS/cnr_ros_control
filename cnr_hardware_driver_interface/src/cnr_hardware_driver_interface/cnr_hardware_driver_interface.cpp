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
#include <rosparam_utilities/rosparam_utilities.h>
#include <cnr_controller_interface_params/cnr_controller_interface_params.h>
#include <cnr_hardware_interface/cnr_robot_hw.h>
#include <configuration_msgs/SendMessage.h>

#include <cnr_hardware_driver_interface/cnr_hardware_driver_interface.h>

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

namespace cnr_hardware_driver_interface
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


RobotHwDriverInterface::~RobotHwDriverInterface()
{
  CNR_TRACE_START(m_logger, "Shutting Down the nodelet...");
  try
  {
    CNR_INFO(m_logger, "Reset the HW (watchdog: 10sec)");

    m_cmi->stopUnloadAllControllers(ros::Duration(10.0));
    /*
    std::vector<std::string> ctrls;
    if (ros::param::has(cnr::control::ctrl_list_param_name(m_hw_name)))
    {
      ros::param::get(cnr::control::ctrl_list_param_name(m_hw_name), ctrls);
      for (const auto ctrl : ctrls)
      {
        std::string last_status;
        ros::param::get(cnr::control::ctrl_last_status_param_name(m_hw_name, ctrl), last_status);
        if (last_status != "UNLOADED")
        {
          CNR_INFO(m_logger, "Last Status Tracked: " << last_status);
          if (!m_cmi->unloadController(ctrl, ros::Duration(1.0)))
          {
            CNR_INFO(m_logger, "Error in unloading the controller " << ctrl);
          }
        }
      }
    }
    */

    CNR_WARN(m_logger, "Join the diagnostic thread");
    m_stop_diagnostic_thread = true;
    if (m_diagnostics_thread.joinable())
    {
      m_diagnostics_thread.join();
    }

    CNR_WARN(m_logger, "Join the update loop");
    if(!stop())
    {
      CNR_FATAL(m_logger, "Error in stopping the control loop!!!");
    }
    
    // m_callback_queue.disable();
    // m_hw_nh.shutdown();
    m_cmi.reset();
    m_cm.reset();
    m_hw.reset();
    m_cnr_hw = nullptr;
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

  CNR_DEBUG(m_logger, "\n\n"+ cnr_logger::BOLDMAGENTA()+ "============= " + m_hw_name 
    + " : driver desctructor ==========" + cnr_logger::RESET() + "\n\n");
}

bool RobotHwDriverInterface::init(const std::string& hw_name, const std::map<std::string, std::string>& remappings)
{
  m_hw_nh = ros::NodeHandle(hw_name, remappings);
  m_root_nh = ros::NodeHandle(ros::names::parentNamespace(hw_name), remappings);

  m_hw_nh.setCallbackQueue(&m_callback_queue);
  //m_root_nh.setCallbackQueue(&m_callback_queue);

  m_hw_namespace = m_hw_nh.getNamespace();
  m_hw_name      = extractRobotName(m_hw_namespace);
  
  m_logger.reset(new cnr_logger::TraceLogger());
  std::string what;
  if( !m_logger->init("NL_" + m_hw_name, m_hw_namespace, false, true, &what))
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ <<": error in creating the logger" << std::endl;
    std::cerr << what << std::endl;
    return false;
  }
  CNR_DEBUG(m_logger, "\n\n"+ cnr_logger::BOLDMAGENTA()+ "============= " + m_hw_name 
    + " : driver initialization ==========" + cnr_logger::RESET() + "\n\n");
  CNR_TRACE_START(m_logger);
  double sampling_period = 0.001;
  if (!rosparam_utilities::get(m_hw_nh.getNamespace() +"/sampling_period", sampling_period,what,&sampling_period))
  {
    CNR_WARN(m_logger, m_hw_namespace + "/sampling_period' does not exist, set equal to 0.001");
    sampling_period = 1.0e-3;
  }

  m_period = ros::Duration(sampling_period);
  dumpState(cnr_hardware_interface::UNLOADED);
  realtime_utilities::DiagnosticsInterface::init(m_hw_name, "RobotHwDriverInterface", "Main Loop");
  realtime_utilities::DiagnosticsInterface::addTimeTracker("cycle",sampling_period);
  realtime_utilities::DiagnosticsInterface::addTimeTracker("read",sampling_period);
  realtime_utilities::DiagnosticsInterface::addTimeTracker("write",sampling_period);
  realtime_utilities::DiagnosticsInterface::addTimeTracker("update",sampling_period);
  
  m_robot_hw_loader.reset(
    new pluginlib::ClassLoader<hardware_interface::RobotHW>("hardware_interface", "hardware_interface::RobotHW"));

  std::string robot_type;
  try
  {
    //==========================================================
    // LOAD THE ROBOTHW
    std::string what;
    if (!rosparam_utilities::get(m_hw_nh.getNamespace() +"/type", robot_type, what))
    {
      CNR_FATAL(m_logger, what );
      CNR_RETURN_FALSE(m_logger);
    }

    CNR_DEBUG(m_logger, "Loading instance: '"  << robot_type << "'");
    CNR_DEBUG(m_logger, "Is Class Available? " << (m_robot_hw_loader->isClassAvailable(robot_type) ? "YES" : "NO"));
    CNR_DEBUG(m_logger, "Name of the class ? " <<  m_robot_hw_loader->getName(robot_type));

    m_robot_hw_loader->loadLibraryForClass(robot_type);
    auto hw = m_robot_hw_loader->createInstance(robot_type);
    m_hw = cnr_hardware_interface::to_std_ptr( hw );
    if (m_hw == nullptr)
    {
      dumpState(cnr_hardware_interface::ERROR);
      CNR_RETURN_FALSE(m_logger, "The RobotHw has not been properly initialized in the doOnInit() function. Abort.");
    }
    m_cnr_hw = dynamic_cast<cnr_hardware_interface::RobotHW*>(m_hw.get()); // if not null, there are many fancy & funny functions
    dumpState( m_cnr_hw ? m_cnr_hw->getState() : cnr_hardware_interface::CREATED );

    if (!m_hw->init(m_root_nh, m_hw_nh))
    {
      dumpState(cnr_hardware_interface::ERROR);
      CNR_RETURN_FALSE(m_logger, "The RobotHw '" + m_hw_name + "'Initialization failed. Abort.");
    }
    dumpState( m_cnr_hw ? m_cnr_hw->getState() : cnr_hardware_interface::INITIALIZED);
    
    if ( m_cnr_hw && (m_cnr_hw->getRobotHwNamespace() != m_hw_namespace) )
    {
      CNR_WARN(m_logger, "Mismatch between the namespace of the RobotHw '" + m_cnr_hw->getRobotHwNamespace()
                + "'and of the hardware label from configuration manager '" + m_hw_namespace + "'. Is it right?");
    }
    //==========================================================

    //==========================================================
    // CREATE THE CONTROLLER MANAGER
    m_cm.reset(new controller_manager::ControllerManager( m_hw.get(), m_hw_nh));
    
    // CREATE THE CONTROLLER MANAGER INTERFACE FROM THE ControllerManager
    m_cmi.reset(new cnr_controller_manager_interface::ControllerManagerInterface(m_logger, m_hw_name, m_cm.get()));
    //==========================================================
  }
  catch (pluginlib::PluginlibException& ex)
  {
    dumpState(cnr_hardware_interface::ERROR);
    const std::vector<std::string>& classes = m_robot_hw_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    CNR_ERROR(m_logger, "Exception while loading '" << robot_type << "': " << ex.what() << "\n Available plugins: " << ss.str());
    CNR_RETURN_FALSE(m_logger);
  }
  catch (std::exception& e)
  {
    dumpState(cnr_hardware_interface::ERROR);
    CNR_ERROR(m_logger, m_hw_name << ": ExitOnInit failed. Exception:" << e.what() );
    CNR_RETURN_FALSE(m_logger);
  }
  CNR_RETURN_TRUE(m_logger);
}



void RobotHwDriverInterface::diagnosticsThread()
{
  CNR_INFO(m_logger, "Diagnostics Thread Started");
  diagnostic_updater::Updater   updater(m_hw_nh, ros::NodeHandle("~"), "/" + m_hw_name);

  updater.setHardwareID(m_hw_name);
  std::string id = "RobotHW | ";

  realtime_utilities::DiagnosticsInterfacePtr            hw_d  = std::dynamic_pointer_cast<realtime_utilities::DiagnosticsInterface>(m_hw);
  realtime_utilities::DiagnosticsInterface*              hwn_d = dynamic_cast<realtime_utilities::DiagnosticsInterface*>(this);

  updater.add(id + "Info"      , hw_d.get(), &cnr_hardware_interface::RobotHW::diagnosticsInfo);
  updater.add(id + "Warning"   , hw_d.get(), &cnr_hardware_interface::RobotHW::diagnosticsWarn);
  updater.add(id + "Error"     , hw_d.get(), &cnr_hardware_interface::RobotHW::diagnosticsError);
  updater.add(id + "Timers"    , hw_d.get(), &cnr_hardware_interface::RobotHW::diagnosticsPerformance);
  updater.add(id + "Main Loop (nodelet)", hwn_d, &RobotHwDriverInterface::diagnosticsPerformance);

  id = "Ctrl | ";
  updater.add(id + "Info"    , m_cmi.get(), &cnr_controller_manager_interface::ControllerManagerInterface::diagnosticsInfo);
  updater.add(id + "Warning" , m_cmi.get(), &cnr_controller_manager_interface::ControllerManagerInterface::diagnosticsWarn);
  updater.add(id + "Error"   , m_cmi.get(), &cnr_controller_manager_interface::ControllerManagerInterface::diagnosticsError);
  updater.add(id + "Timers"  , m_cmi.get(), &cnr_controller_manager_interface::ControllerManagerInterface::diagnosticsPerformance);

  ros::WallDuration wd(updater.getPeriod());
  m_diagnostics_thread_running = true;
  while (ros::ok() && !m_stop_diagnostic_thread)
  {
    updater.update();
    wd.sleep();
  }
  m_diagnostics_thread_running = false;
  CNR_DEBUG(m_logger, "Diagnostic finished.");
  return;
}
bool RobotHwDriverInterface::start(const ros::Duration& watchdog) 
{
  CNR_TRACE_START(m_logger);
  m_stop_run = false; 
  m_thread_run = std::thread(&RobotHwDriverInterface::run, this);
  ros::Time start = ros::Time::now();
  while(ros::ok())
  {
    if((ros::Time::now()-start) > watchdog)
    {
      CNR_RETURN_FALSE(m_logger);
    }
    if(retriveState() == cnr_hardware_interface::RUNNING)
    {
      CNR_WARN(m_logger, "RobotHW RT-Control Loop Started!");
      break;
    }
  }
  CNR_RETURN_TRUE(m_logger);
}
bool RobotHwDriverInterface::stop(const ros::Duration& watchdog)
{
  CNR_TRACE_START(m_logger);
  m_stop_run = true;
  ros::Time start = ros::Time::now();
  if( m_thread_run.joinable() )
  {
    CNR_WARN(m_logger, "Waiting for joining the run() thread");
    m_thread_run.join();
  }
  else
  {
    while(ros::ok())
    {
      CNR_WARN(m_logger, "Waiting for stopping the run()");
      if((ros::Time::now()-start) > watchdog)
      {
        CNR_ERROR(m_logger, "The thread did not stopped within the expected watchdog. Abort");
        CNR_RETURN_FALSE(m_logger);
      }
      if(retriveState()!=cnr_hardware_interface::RUNNING)
      {
        CNR_WARN(m_logger, "RobotHW RT-Control Loop Ended!");
        break;
      }
    }
  }
  CNR_RETURN_TRUE(m_logger);
}
/**
 * 
 * 
 * 
 * 
 * 
 */
void RobotHwDriverInterface::run()
{
  CNR_TRACE_START(m_logger);
  std::string error;
  if(!fetchState(error))
  {
    CNR_ERROR(m_logger, "Fetching the state failed: '" << error << "'");  
    CNR_RETURN_NOTOK(m_logger, void());
  }
  if(retriveState()==cnr_hardware_interface::RUNNING)
  {
    CNR_ERROR(m_logger, "The Driver is already in running state.. have you called both 'start' and 'run'?");  
    CNR_RETURN_NOTOK(m_logger, void());
  }

  CNR_DEBUG(m_logger, "Supported Hardware Interfaces:");
  for (auto& elem : m_hw->getNames())
  {
    CNR_DEBUG(m_logger, "[ " << m_hw_name << " ] - '" <<  elem << "'");
  }

  CNR_WARN(m_logger, "Start update thread (period: " << m_period << ")");    
  if(m_cnr_hw)
  {
    CNR_DEBUG(m_logger, "Start the Diagnostic thread");
    CNR_DEBUG(m_logger, "Frequency: '" << 1.0 / m_period.toSec() << "'");

    m_diagnostics_thread_running = false;
    m_stop_diagnostic_thread     = false;
    ros::Time start              = ros::Time::now();
    m_diagnostics_thread         = std::thread(&cnr_hardware_driver_interface::RobotHwDriverInterface::diagnosticsThread, this);
    while(!m_diagnostics_thread_running)
    {
      if ((ros::Time::now() - start).toSec() > 10.0)
      {
        CNR_RETURN_NOTOK(m_logger, void(), "Timeout Exipred. Main Thread did not started yet. Abort.");
      }
      ros::Duration(0.05).sleep();
    }
  }

  if(!m_cnr_hw)
  {
    dumpState(cnr_hardware_interface::RUNNING);
  }

#if PREEMPTIVE_RT
  realtime_utilities::period_info  pinfo;
  if(!realtime_utilities::rt_init_thread(RT_STACK_SIZE, sched_get_priority_max(SCHED_RR)-2, SCHED_RR, &pinfo, m_period.toNSec()) )
  {
    CNR_ERROR(m_logger, "Failed in setting thread rt properties. Exit. ");
    std::raise(SIGINT);
    return;
  }
#endif

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

  m_stop_run = false;

  try
  {
    if(m_cnr_hw && !m_cnr_hw->initRT())
    {
      CNR_ERROR_THROTTLE(m_logger, 5.0, "Error in RT init. Abort");
      dumpState(cnr_hardware_interface::ERROR);
      CNR_RETURN_NOTOK(m_logger, void());
    }
  }
  catch (std::exception& e)
  {
    CNR_ERROR(m_logger, "updateThread error call hardware interface initRT(): " << e.what());
    dumpState(cnr_hardware_interface::ERROR);
    CNR_RETURN_NOTOK(m_logger, void());
  }


  while (ros::ok() && !m_stop_run)
  {
#if defined(USE_TIMER_REALTIME_UTILS)
  realtime_utilities::timer_wait_rest_of_period(&(ptarget.next_period));
  realtime_utilities::timer_inc_period(&ptarget);
#elif defined(USE_TIMERFD)
  wait_period(&info);
#elif defined(USE_WALLRATE)
  wr.sleep();
#endif

    m_callback_queue.callAvailable();

    timeSpanStrakcer("cycle")->time_span();
    if (m_stop_run)
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
      dumpState(cnr_hardware_interface::ERROR);
      CNR_RETURN_NOTOK(m_logger, void());
    }

    try
    {
      timeSpanStrakcer("update")->tick();
      // 
      // it executes the 
      // hw->doSwitch() as needed, and the update of the control strategies
      //
      m_cmi->update(ros::Time::now(), m_period);
      timeSpanStrakcer("update")->tock();
    }
    catch (std::exception& e)
    {
      CNR_WARN(m_logger, "updateThread error call controller manager update(): " << std::string(e.what()));
      dumpState(cnr_hardware_interface::ERROR);
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
      CNR_ERROR(m_logger, "updateThread error call hardware interface write() " << e.what());
      dumpState(cnr_hardware_interface::ERROR);
      CNR_RETURN_NOTOK(m_logger, void());
    }

    // if (m_cmi)
    // {
    //   std::vector<std::string> running_controllers = m_cmi->getControllerNames();
    //   bool ok = true;
    //   for(const auto & ctrl_name : running_controllers)
    //   {
    //     controller_interface::ControllerBase* ctrl = m_cmi->getControllerByName(ctrl_name);
    //     if(ctrl)
    //     {
    //       bool _aborted = (ctrl->state_ == controller_interface::ControllerBase::ControllerState::ABORTED);
    //       if(_aborted)
    //       {
    //         CNR_ERROR(m_logger, "The controller '"<< ctrl_name <<"' is not in RUNNING state! The state is "
    //           << ( ctrl->state_ == controller_interface::ControllerBase::ControllerState::CONSTRUCTED ? "CONSTRUCTED"
    //              : ctrl->state_ == controller_interface::ControllerBase::ControllerState::INITIALIZED ? "INITIALIZED"
    //              : ctrl->state_ == controller_interface::ControllerBase::ControllerState::INITIALIZED ? "RUNNING"
    //              : ctrl->state_ == controller_interface::ControllerBase::ControllerState::STOPPED     ? "STOPPED"
    //              : ctrl->state_ == controller_interface::ControllerBase::ControllerState::WAITING     ? "WAITING"
    //              : "ABORTED") );
    //       }
    //       ok &= !_aborted;
    //     }
    //   }
    //   if(!ok) 
    //   {
    //     dumpState(cnr_hardware_interface::ERROR);
    //     CNR_RETURN_NOTOK(m_logger, void())
    //   };
    // }

    if (m_cnr_hw && m_cnr_hw->getState() == cnr_hardware_interface::ERROR)
    {
      CNR_ERROR_THROTTLE(m_logger, 1.0, "RobotHw is in error");
      dumpState(cnr_hardware_interface::ERROR);
      CNR_RETURN_NOTOK(m_logger, void());
    }
  }

  dumpState(cnr_hardware_interface::SHUTDOWN);
  CNR_WARN(m_logger, "EXIT UPDATE THREAD");
  CNR_RETURN_OK(m_logger, void());
}


//==================================================
bool RobotHwDriverInterface::dumpState(const cnr_hardware_interface::StatusHw& status)
{
  std::string last_status = cnr_hardware_interface::to_string(retriveState());
  if(m_state_history.size() == 0)
  {
    m_state_history.push_back(last_status);
  }
  else
  {
    if(m_state_history.back() != last_status)
    {
      m_state_history.push_back(last_status);
      ros::param::set(hw_last_status_param_name(m_hw_namespace), last_status);
      ros::param::set(hw_status_param_name(m_hw_namespace), m_state_history);
      CNR_DEBUG(m_logger, "RobotHW '" << m_hw_namespace
                            << "' New Status " <<  cnr_hardware_interface::to_string(retriveState()));
    }
  }
  m_state = status;
  return true;
}

bool RobotHwDriverInterface::fetchState(std::string& error)
{
  std::string status;
  if(!ros::param::get(hw_last_status_param_name(m_hw_namespace), status))
  {
    error = "The param status does not exists...";
    return false;
  }
  for (const cnr_hardware_interface::StatusHw& it : cnr_hardware_interface::StatusHwIterator())
  {
    if (status == cnr_hardware_interface::to_string(it))
    {
      m_state = it;
    }
    return true;
  }
  error = "The param status '"+status+"' is unrecongnized...";
  return false;
}
//==================================================


//! It stop and unload the controllers. It does not change the state of the Driver/RobotHW
//! Indeed, the loop in run() still work, without any running controller
bool RobotHwDriverInterface::stopUnloadAllControllers(const ros::Duration& watchdog)
{
  CNR_TRACE_START(m_logger);
  static const std::vector<controller_manager_msgs::ControllerState> vc_empty;
  static const std::vector<std::string> vs_empty;

  std::vector<controller_manager_msgs::ControllerState> running;
  std::vector<controller_manager_msgs::ControllerState> stopped;

  if (!m_cmi->listControllers(running, stopped, watchdog))
  {
    CNR_ERROR(m_logger, m_hw_name << " Error in getting the information of the status of the controllers." 
                          << m_cmi->error() );
    CNR_RETURN_FALSE(m_logger);
  }
  stopped.insert(stopped.end(), running.begin(), running.end());

  if (!m_cmi->switchControllers(vc_empty, running, 1, watchdog))
  {
    CNR_ERROR(m_logger, m_hw_name << " Error in stopping controllers:" 
                      << m_cmi->error() );
    CNR_RETURN_FALSE(m_logger); 
  }

  if (!m_cmi->unloadControllers(stopped, watchdog))
  {
    CNR_ERROR(m_logger, m_hw_name << " Error in unloading controllers:" 
                      << m_cmi->error() );
    CNR_RETURN_FALSE(m_logger); 
  }
  CNR_RETURN_TRUE(m_logger); 
}



bool RobotHwDriverInterface::loadAndStartControllers(const std::vector<std::string>& next_controllers,
                                                      const size_t& strictness, const ros::Duration& watchdog,const std::string& configuration_name)
{
  CNR_TRACE_START(m_logger);
  try
  {
    CNR_DEBUG(m_logger, "\n\n"+ cnr_logger::BOLDMAGENTA()+ "============= CONFIGURATION: " + configuration_name + ", HW: " + m_hw_name 
    + ", load a new set of controllers (strictness: "+ std::to_string(strictness) +") ==========" + cnr_logger::RESET() + "\n\n");
    if(!m_cmi->switchControllers(strictness, next_controllers, watchdog))
    {
      CNR_ERROR(m_logger, m_hw_name << " Error in loading and starting the controllers:" 
                      << m_cmi->error() );
      CNR_RETURN_FALSE(m_logger); 
    }
  }
  catch(std::exception& e)
  {
    CNR_ERROR(m_logger, m_hw_name << "Exception in starting the controllers. Error: " << std::string(e.what())
                    << m_cmi->error() );
    CNR_RETURN_FALSE(m_logger); 
  }
  catch(...)
  {
    CNR_ERROR(m_logger, m_hw_name << "Unhandled Exception in switch controllers."
                    << m_cmi->error() );
    CNR_RETURN_FALSE(m_logger); 
  }
  CNR_RETURN_TRUE(m_logger); 
}

}
