
#include <sstream>
#include <pluginlib/class_list_macros.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <cnr_hardware_interface/cnr_robot_hw.h>
#include <cnr_hardware_nodelet_interface/cnr_robot_hw_nodelet.h>

PLUGINLIB_EXPORT_CLASS(cnr_hardware_nodelet_interface::RobotHwNodelet, nodelet::Nodelet)

namespace cnr_hardware_nodelet_interface
{

inline std::string extractRobotName( const std::string& hw_namespace )
{
  std::string hw_name = hw_namespace;
  hw_name      .erase(0,1);
  std::replace( hw_name.begin(), hw_name.end(), '/', '_');
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

RobotHwNodelet::RobotHwNodelet()
{
}

RobotHwNodelet::~RobotHwNodelet()
{
  CNR_TRACE_START( *m_logger, "Shutting Down the nodelet...");

  try
  {
    CNR_INFO( *m_logger, "Reset the HW (watchdog: 10sec)" );
    std::vector<std::string> ctrls;
    if( ros::param::has( cnr_controller_interface::ctrl_list_param( m_hw_name )) )
    {
      ros::param::get( cnr_controller_interface::ctrl_list_param( m_hw_name ), ctrls);
      for( const auto ctrl: ctrls)
      {
        std::string last_status;
        ros::param::get( cnr_controller_interface::last_status_param( m_hw_name , ctrl), last_status);
        if( last_status != "UNLOADED" )
        {
          CNR_INFO( *m_logger, "Last Status Tracked: " << last_status );
          if( !m_cm->unloadController(ctrl) )
          {
            CNR_INFO( *m_logger, "Error in unloading the controller " << ctrl );
          }
        }
      }
    }

    CNR_WARN( *m_logger, "Join the diagnostic thread");
    m_stop_diagnostic_thread = true;
    if (m_diagnostics_thread.joinable())
    {
      m_diagnostics_thread.join();
    }

    CNR_WARN( *m_logger, "Join the update thread");
    m_stop_update_thread = true;
    if (m_update_thread.joinable())
    {
      m_update_thread.join();
    }

    m_hw.reset();
  }
  catch(std::exception& e)
  {
    CNR_FATAL(*m_logger, "Error in shutting down the RobotHw: " << e.what() );
  }
  catch(...)
  {
    CNR_FATAL(*m_logger, "Error in shuttind down the RobotHw: " );
  }
  CNR_TRACE( *m_logger, "[ DONE] Shutted Down");
}

void RobotHwNodelet::onInit()
{

  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  m_hw_namespace = getPrivateNodeHandle().getNamespace();
  m_hw_name      = extractRobotName(m_hw_namespace);
  m_updater.setHardwareID( m_hw_name );
  m_logger.reset( new cnr_logger::TraceLogger( "NL_" + m_hw_name, m_hw_namespace ));

  try
  {
    CNR_TRACE_START(*m_logger);
    if(enterOnInit( ) && doOnInit( ) && exitOnInit ( ) )
    {
      m_stop_update_thread = m_stop_diagnostic_thread = false;
      CNR_RETURN_OK(*m_logger, void());
    }
  }
  catch( std::exception& e )
  {
    CNR_RETURN_NOTOK( *m_logger, void(), m_hw_name + ": ExitOnInit failed. Exception caught: " +std::string( e.what() ) );
  }
  catch( ... )
  {
    CNR_RETURN_NOTOK( *m_logger, void(), m_hw_name + ": ExitOnInit failed. UNhandled Exception" );
  }
}

bool RobotHwNodelet::doOnInit()
{
  CNR_TRACE_START(*m_logger);
  std::string robot_type;
  try
  {
    if (!m_hw_nh.getParam("type", robot_type ) )
    {
      CNR_FATAL(*m_logger, "The param '"<< m_hw_nh.getNamespace() <<"/type' is missing! Abort.");
      CNR_RETURN_FALSE(*m_logger);
    }

    CNR_DEBUG(*m_logger, "Loading instance: '"<< robot_type <<"'");
    CNR_DEBUG(*m_logger, "Is Class Available? " << ( m_robot_hw_plugin_loader->isClassAvailable(robot_type) ? "YES" : "NO" ) );
    CNR_DEBUG(*m_logger, "Name of the class ? " <<  m_robot_hw_plugin_loader->getName(robot_type));

    m_robot_hw_plugin_loader->loadLibraryForClass(robot_type);

    m_hw = m_robot_hw_plugin_loader->createInstance( robot_type );
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = m_robot_hw_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    CNR_ERROR(*m_logger, "Exception while loading '" << robot_type << "': " << ex.what() << "\n Available plugins: " << ss.str());
  }
  catch( std::exception& e )
  {
    CNR_RETURN_FALSE(*m_logger, "OnInit of FakeRobotHw failed! what: " + std::string( e.what() ));
  }
  CNR_RETURN_TRUE(*m_logger);
}

bool RobotHwNodelet::enterOnInit()
{
  CNR_TRACE_START(*m_logger);
  m_stop_update_thread = m_stop_diagnostic_thread = false;

  // getNodeHandle(), parent nodelet handle (typically "/")  --- getPrivateNodeHandle() -> /nodelet_name/
  m_nh     = getNodeHandle();
  m_hw_nh  = getPrivateNodeHandle();

  double sampling_period = 0.001;
  if (!m_hw_nh.getParam("sampling_period", sampling_period))
  {
    CNR_WARN( *m_logger, m_hw_namespace+"/sampling_period' does not exist, set equal to 0.001");
    sampling_period = 1.0e-3;
  }

  m_period              = ros::Duration( sampling_period );

  m_time_span_tracker.emplace( "cycle" , new realtime_utilities::TimeSpanTracker( int( 10.0 / sampling_period ), sampling_period ) );
  m_time_span_tracker.emplace( "read"  , new realtime_utilities::TimeSpanTracker( int( 10.0 / sampling_period ), sampling_period ) );
  m_time_span_tracker.emplace( "write" , new realtime_utilities::TimeSpanTracker( int( 10.0 / sampling_period ), sampling_period ) );
  m_time_span_tracker.emplace( "update", new realtime_utilities::TimeSpanTracker( int( 10.0 / sampling_period ), sampling_period ) );

  m_robot_hw_plugin_loader.reset(
        new pluginlib::ClassLoader<cnr_hardware_interface::RobotHW>("cnr_hardware_interface", "cnr_hardware_interface::RobotHW"));

  CNR_RETURN_TRUE(*m_logger);
}

bool RobotHwNodelet::exitOnInit()
{
  CNR_TRACE_START(*m_logger);
  CNR_DEBUG( *m_logger, "Triggering the start of the main thread");

  if( m_hw == nullptr )
  {
    CNR_RETURN_FALSE(*m_logger, "The RobotHw has not been properly initialized in the doOnInit() function. Abort.");
  }

  if (!m_hw->init(m_nh,m_hw_nh))
  {
    CNR_RETURN_FALSE(*m_logger, "The RobotHw '" + m_hw_name + "'Initialization failed. Abort.");
  }

  if( m_hw->getRobotHwNamespace() != m_hw_namespace )
  {
    CNR_WARN( *m_logger, "Mismatch between the namespace of the RobotHw '" + m_hw->getRobotHwNamespace()
              + "'and of the hardware interface label from configuration manager '" + m_hw_namespace + "'. Is it right?");
  }

  m_cm.reset(new controller_manager::ControllerManager ( m_hw.get(), m_hw_nh ) );

  m_update_thread_state = ON_INIT;
  ros::Time start       = ros::Time::now();
  m_update_thread       = std::thread(&cnr_hardware_nodelet_interface::RobotHwNodelet::controlUpdateThread, this);
  while( m_update_thread_state != RUNNING )
  {
    if( m_update_thread_state == ON_ERROR )
    {
      CNR_RETURN_FALSE(*m_logger, "The update thread is in Error. Abort.");
    }
    if( m_update_thread_state == EXPIRED )
    {
      CNR_RETURN_FALSE(*m_logger, "The update thread is expired. Something went wrong during the start. Abort.");
    }
    if( ( ros::Time::now() - start ).toSec() > 10.0 )
    {
      CNR_RETURN_FALSE(*m_logger, "Timeout Expired. The update thread did not yet started. Abort.");
    }
    ros::Duration(0.05).sleep();
  }

  CNR_DEBUG(*m_logger, "Diagnostic started "                          );
  CNR_DEBUG(*m_logger, "Frequency: '" << 1.0/m_period.toSec() << "'"  );
  CNR_DEBUG(*m_logger, "Supported Hardware Interfaces:"               );
  for (auto& elem: m_hw->getNames())
  {
    CNR_DEBUG(*m_logger, "[ " << m_hw_name << " ] - '" <<  elem << "'" );
  }

  m_diagnostics_thread_state = ON_INIT;
  start                      = ros::Time::now();
  m_diagnostics_thread       = std::thread(&cnr_hardware_nodelet_interface::RobotHwNodelet::diagnosticsThread, this);
  while( m_diagnostics_thread_state != RUNNING )
  {
    if( m_diagnostics_thread_state == ON_ERROR )
    {
      CNR_RETURN_FALSE( *m_logger, "Main thread in ERROR" );
    }
    if( ( ros::Time::now() - start ).toSec() > 10.0 )
    {
      CNR_RETURN_FALSE( *m_logger, "Timeout Exipred. Main Thread did not started yet. Abort." );
    }
    ros::Duration(0.05).sleep();
  }
  CNR_DEBUG( *m_logger, "The main is running");
  CNR_RETURN_TRUE( *m_logger );
}

void RobotHwNodelet::diagnostics( diagnostic_updater::DiagnosticStatusWrapper &stat )
{

  stat.hardware_id = m_hw_name;
  stat.level       = diagnostic_msgs::DiagnosticStatus::OK;
  stat.name        = m_hw_name;
  stat.message     = "RobotHW Cycle Time Statistics";
  for( auto const & ts : m_time_span_tracker )
  {
    diagnostic_msgs::KeyValue k; k.key = ts.first + " [s]";
    k.value = to_string( ts.second->getMean() )
             + std::string(" [ ") + to_string( ts.second->getMin() ) + " - " + to_string( ts.second->getMax() ) + std::string(" ] ")
             + std::string("Missed: ")  + to_string( ts.second->getMissedCycles( ) );
    stat.add( k.key, k.value );
  }
}

void RobotHwNodelet::diagnosticsThread()
{
  CNR_INFO(*m_logger, "Diagnostics Thread Started");
  diagnostic_updater::Updater   updater( m_hw_nh, ros::NodeHandle("~"), "/" + m_hw_name );

  updater.setHardwareID( m_hw_name );
  updater.add( m_hw_name + " Status Updater"    , m_hw.get(), &cnr_hardware_interface::RobotHW::diagnostics );
  updater.add( m_hw_name + " Statistics Updater", this      , &RobotHwNodelet::diagnostics );

  ros::WallDuration wd( updater.getPeriod() );
  try
  {
    m_diagnostics_thread_state = RUNNING;

    while (ros::ok() && !m_stop_diagnostic_thread)
    {
      updater.update();
      wd.sleep();
    }
    CNR_DEBUG(*m_logger, "Diagnostic finished.");
  }
  catch(std::exception& e)
  {
    m_stop_diagnostic_thread = true;
    CNR_FATAL( *m_logger, "Hardware interface is in error state. Exception: " <<  e.what() );
    m_diagnostics_thread_state = ON_ERROR;
  }

  bool hardware_interface_with_error = ( m_hw == nullptr ) ||  (m_hw->getStatus()==cnr_hardware_interface::ERROR);
  if ( hardware_interface_with_error )
  {
    CNR_FATAL( *m_logger, "The Hardware interface '" << m_hw_name <<"' is in error state, shutting down" );
  }

  m_diagnostics_thread_state = EXPIRED;
  CNR_WARN( *m_logger, "Diagnositcs Thread Expired" );
  return;  
  
}


void RobotHwNodelet::controlUpdateThread()
{
  CNR_WARN( *m_logger, "Start update thread (period: " << m_period << ")" );
  ros::WallRate wr(m_period);
  m_update_thread_state = RUNNING;
  while ( ros::ok() )
  {
    m_time_span_tracker.at("cycle")->time_span();

    if(m_stop_update_thread)
    {
      CNR_WARN( *m_logger, "Exiting update thread of the hardware interface because a stop has been triggered.");
      break;
    }

    try
    {
      m_time_span_tracker.at("read")->tick();
      m_hw->read(ros::Time::now(),m_period);
      m_time_span_tracker.at("read")->tock();

    }
    catch(std::exception& e)
    {
      CNR_ERROR( *m_logger, "updateThread error call hardware interface read(): " <<e.what());
      break;
    }

    try
    {
      m_time_span_tracker.at("update")->tick();
      m_cm->update(ros::Time::now(), m_period);
      m_time_span_tracker.at("update")->tock();
    }
    catch(std::exception& e)
    {
      CNR_WARN( *m_logger, "updateThread error call controller manager update(): " << std::string( e.what() ));
      m_stop_update_thread = true;
      m_update_thread_state = ON_ERROR;
      return;
    }
    try
    {
      m_time_span_tracker.at("write")->tick();
      m_hw->write(ros::Time::now(),m_period);
      m_time_span_tracker.at("write")->tock();
    }
    catch(std::exception& e)
    {
      CNR_WARN( *m_logger, "updateThread error call hardware interface write() " << e.what());
    }
    
    if (m_hw->getStatus() == cnr_hardware_interface::ERROR)
    {
      CNR_ERROR_THROTTLE( *m_logger, 5.0, "RobotHw is in error");
    }

    wr.sleep();
  }

  m_stop_update_thread = true;
  m_update_thread_state = EXPIRED;
  CNR_WARN( *m_logger, "EXIT UPDATE THREAD");
}
 
}
