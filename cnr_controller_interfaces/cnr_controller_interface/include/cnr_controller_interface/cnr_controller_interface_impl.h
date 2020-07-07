#ifndef __CNR_CONTROLLER_INFERFACE__IMPL__H__
#define __CNR_CONTROLLER_INFERFACE__IMPL__H__

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <cnr_controller_interface/cnr_controller_interface.h>


namespace cnr_controller_interface
{

template< class T >
Controller< T >::~Controller( )
{
    CNR_TRACE_START(*m_logger);
    m_controller_nh_callback_queue.callAvailable( ros::WallDuration( 5.0 ) );
    for( auto & t : m_sub )
    {
        t.second.sub.shutdown();
    }
    for( auto & t : m_pub )
    {
        t.second.pub.shutdown();
    }
    
    dump_state("UNLOADED");
    CNR_TRACE(*m_logger, "[ DONE]");
    m_logger.reset();
}

/**
*
*
*
* Base class to log the controller status
*/
template< class T >
bool  Controller< T >::init( T* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) 
{
    
    m_hw_name     = root_nh.getNamespace();
    m_ctrl_name   = controller_nh.getNamespace();
    if (m_ctrl_name.find(m_hw_name) != std::string::npos) 
    {
        m_ctrl_name.erase(m_ctrl_name.find(m_hw_name), m_hw_name.length());
    }
    else
    {
        ROS_FATAL_STREAM("Controller configurations seems broken. Root nh: " << root_nh.getNamespace() << " Controller nh: " <<  controller_nh.getNamespace() );
        return false;
    }
        
    std::replace( m_hw_name.begin()  , m_hw_name.end()  , '/', '_');  if(m_hw_name  .at(0) == '_') m_hw_name  .erase(0,1);
    std::replace( m_ctrl_name.begin(), m_ctrl_name.end(), '/', '_');  if(m_ctrl_name.at(0) == '_') m_ctrl_name.erase(0,1);

    m_logger.reset( new cnr_logger::TraceLogger( m_hw_name + "-" + m_ctrl_name ) );
    if(!m_logger->init( controller_nh.getNamespace(), true, false ) )
    {
      if(!m_logger->init( root_nh.getNamespace(), true, false ) )
      {
        return false;
      }
    }
    
    CNR_TRACE_START(*m_logger);

    m_root_nh       = root_nh;
    m_controller_nh = controller_nh; // handle to callback and remapping
    m_hw            = hw;
    
    if( !m_root_nh.getParam("sampling_period", m_sampling_period ) )
    {
        CNR_RETURN_FALSE(*m_logger, "The parameter '"+ m_root_nh.getNamespace() + "/sampling_period' is not set. Abort");
    }
        
    int maximum_missing_cycles = 10;
    if( !m_controller_nh.getParam("watchdog", m_watchdog ) )
    {
      if( !m_controller_nh.getParam("maximum_missing_cycles", maximum_missing_cycles ) )
      {
        if( !m_root_nh.getParam("watchdog", m_watchdog ) )
        {
          if( !m_root_nh.getParam("maximum_missing_cycles", maximum_missing_cycles ) )
          {
            m_watchdog = maximum_missing_cycles * m_sampling_period;
            CNR_WARN(*m_logger, "Neither 'watchdog' and 'maximum_missing_cycles' are in the param server"
                            << "the watchdog is super-imposed to "<< std::to_string(maximum_missing_cycles) << " times the sampling period, and it results in " <<m_watchdog);
          }
          else
          {
              m_watchdog = maximum_missing_cycles * m_sampling_period;
          }
        }
      }
    }
    CNR_DEBUG(*m_logger, "Watchdog: " <<m_watchdog);
    m_controller_nh.setCallbackQueue(&m_controller_nh_callback_queue);
    m_status_history.clear();

    if( enterInit() && doInit( ) && exitInit() )
    {
        CNR_RETURN_TRUE( *m_logger );
    }

    m_diagnostics_pub = m_controller_nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",1);

    CNR_RETURN_BOOL( *m_logger, dump_state( )) ;
}

template< class T >
void Controller< T >::starting(const ros::Time& time) 
{
    CNR_TRACE_START(*m_logger);
    if( enterStarting( ) && doStarting( time ) && exitStarting ( ) )
    {
        CNR_RETURN_OK( *m_logger, void(), "Starting Ok! Ready to go!");
    }
    else
    {
        CNR_ERROR(*m_logger, "The starting of the controller failed. Abort Request to ControllerManager sent.");
        controller_interface::Controller< T >::abortRequest(time);
        CNR_RETURN_NOTOK( *m_logger, void());
    }
}

template< class T >
void Controller< T >::update(const ros::Time& time, const ros::Duration& period) 
{
    CNR_TRACE_START_THROTTLE(*m_logger, 10.0);
    if(enterUpdate( ) && doUpdate( time, period ) && exitUpdate( ) )
    {
        CNR_RETURN_OK_THROTTLE( *m_logger, void(), 10.0);
    }
    else
    {
        CNR_ERROR( *m_logger, "Error in update, stop request called to stop the controller quietly.");
        controller_interface::Controller< T >::stopRequest(time);
        CNR_RETURN_NOTOK_THROTTLE( *m_logger, void(), 10.0);
    }
}

template< class T >
void Controller< T >::stopping(const ros::Time& time) 
{
    CNR_TRACE_START(*m_logger);
    if(enterStopping( ) && doStopping( time ) && exitStopping ( ) )
    {
    CNR_RETURN_OK( *m_logger, void());
    }
    else
    {
    controller_interface::Controller< T >::abortRequest(time);
    CNR_RETURN_NOTOK( *m_logger, void());
    }
}

template< class T >
void Controller< T >::waiting(const ros::Time& time) 
{
    CNR_TRACE_START(*m_logger);

    if(enterWaiting( ) && doWaiting( time ) && exitWaiting ( ) )
    {
        CNR_RETURN_OK( *m_logger, void());
    }
    else
    {
        controller_interface::Controller< T >::abortRequest(time);
        CNR_RETURN_NOTOK( *m_logger, void());
    }
}

template< class T >
void Controller< T >::aborting(const ros::Time& time) 
{
    CNR_TRACE_START(*m_logger);
    CNR_EXIT_EX(*m_logger, enterAborting( ) && doAborting( time )  && exitAborting ( ) );
}

/////////////////////////////////////

template< class T >
bool Controller< T >::enterInit( )
{
    CNR_TRACE_START(*m_logger);
    CNR_RETURN_BOOL(*m_logger, dump_state( ) );
}

template< class T >
bool Controller< T >::exitInit( )
{
    CNR_TRACE_START(*m_logger);
    CNR_RETURN_BOOL(*m_logger, dump_state( ) );
}

template< class T >
bool Controller< T >::enterStarting( )
{
    CNR_TRACE_START(*m_logger);
    CNR_RETURN_BOOL(*m_logger, dump_state( ) );
}

template< class T >
bool Controller< T >::exitStarting ( )   
{ 
    CNR_TRACE_START(*m_logger);
    if( !callAvailable( m_watchdog) )
    {
        CNR_WARN(*m_logger, "The callback is still not available..." );  
    }
    CNR_RETURN_BOOL(*m_logger, dump_state( ) );
}

template< class T >
bool Controller< T >::enterUpdate  ( )   
{ 
    CNR_TRACE_START_THROTTLE(*m_logger, 10.0);
    if( !callAvailable( m_watchdog) )
    {
        dump_state( "CALLBACK_TIMEOUT_ERROR" );
        CNR_RETURN_FALSE_THROTTLE(*m_logger, 5.0 );  
    }
    CNR_RETURN_BOOL_THROTTLE(*m_logger, 10.0, dump_state( ) )
}

template< class T >
bool Controller< T >::exitUpdate   ( )
{
    return dump_state( ); 
}

template< class T >
bool Controller< T >::enterStopping( )
{
    return dump_state( );
}

template< class T >
bool Controller< T >::exitStopping ( )
{
    m_controller_nh_callback_queue.callAvailable( ros::WallDuration( 5.0 ) );
    for( auto & t : m_sub )
    {
        t.second.sub.shutdown();
    }
    for( auto & t : m_pub )
    {
        t.second.pub.shutdown();
    }
    m_controller_nh_callback_queue.disable(); 
    return dump_state( "STOPPED" );
}

template< class T >
bool Controller< T >::enterWaiting ( )
{ 
    return dump_state( );
}
    
template< class T >
bool Controller< T >::exitWaiting  ( )  
{ 
    m_controller_nh_callback_queue.callAvailable( ros::WallDuration( 5.0 ) );
    for( auto & t : m_sub )
    {
        t.second.sub.shutdown();
    }
    for( auto & t : m_pub )
    {
        t.second.pub.shutdown();
    }
    return dump_state( );
}

template< class T >
bool Controller< T >::enterAborting( ) 
{
    return dump_state( );
}
    
template< class T >
bool Controller< T >::exitAborting ( )
{
    m_controller_nh_callback_queue.callAvailable( ros::WallDuration( 5.0 ) );
    for( auto & t : m_sub )
    {
        t.second.sub.shutdown();
    }
    for( auto & t : m_pub )
    {
        t.second.pub.shutdown();
    }
    m_controller_nh_callback_queue.disable(); 
    return dump_state( );
}

template< class T >
bool Controller< T >::dump_state( const std::string& status )
{

  if( m_status_history.size() == 0 )
  {
    m_status_history.push_back( status );
  }
  else
  {
  if( m_status_history.back() != status )
  {
    m_status_history.push_back(status);
    m_controller_nh.setParam( cnr_controller_interface::last_status_param(m_hw_name, m_ctrl_name), status );
    m_controller_nh.setParam( cnr_controller_interface::status_param(m_hw_name, m_ctrl_name), m_status_history );
  }
  }

  if( m_diagnostic.status.size() > 0  )
  {
    m_diagnostics_pub.publish(m_diagnostic);
    m_diagnostic.status.clear();
  }
  return true;
}

template< class T >
bool Controller< T >::dump_state( )
{
    std::string last_status = controller_interface::Controller< T >::isAborted()     ?  "ABORTED"
                            : controller_interface::Controller< T >::isInitialized() ?  "INITIALIZED"
                            : controller_interface::Controller< T >::isRunning()     ?  "RUNNING"
                            : controller_interface::Controller< T >::isWaiting()     ?  "WAITING"
                            : controller_interface::Controller< T >::isStopped()     ?  "STOPPED"
                            : "CONSTRUCTED";
    return dump_state( last_status );
}

template< class T >
void Controller< T >::add_diagnostic_message( const std::string& msg, const std::string& name, const std::string& level, const bool verbose)
{
    diagnostic_msgs::DiagnosticStatus diag;
    diag.name       = name;
    diag.hardware_id= m_hw_name;
    diag.message    = " [ "+ m_hw_name +" ] " + msg;

    if( level == "OK"   ) { diag.level = diagnostic_msgs::DiagnosticStatus::OK;     CNR_INFO_COND( *m_logger, verbose, "[" << m_hw_name << " ] " << msg );}
    if( level == "WARN" ) { diag.level = diagnostic_msgs::DiagnosticStatus::WARN;   CNR_WARN_COND( *m_logger, verbose, "[" << m_hw_name << " ] " << msg );}
    if( level == "ERROR" ){ diag.level = diagnostic_msgs::DiagnosticStatus::ERROR;  CNR_ERROR_COND(*m_logger, verbose, "[" << m_hw_name << " ] " << msg );}
    if( level == "STALE" ){ diag.level = diagnostic_msgs::DiagnosticStatus::STALE;  CNR_INFO_COND( *m_logger, verbose, "[" << m_hw_name << " ] " << msg );}

    m_diagnostic.status.push_back(diag);
}



template<class T> template<class M>
void Controller< T >::add_publisher (const std::string& id, const std::string &topic, uint32_t queue_size, bool latch )
{
  m_pub[id].pub   = m_controller_nh.advertise< M >(topic, queue_size, latch);
  m_pub[id].start = nullptr;
}

template<class T> template<class M>
bool Controller< T >::publish (const std::string& id, const M &message)
{
    m_pub[id].pub.publish(message);
    auto n = std::chrono::high_resolution_clock::now();
    if( m_pub[id].start == nullptr )
    {
      m_pub[id].start = new std::chrono::high_resolution_clock::time_point();
      *m_pub[id].start = n;
      m_pub[id].last   = n;
    }
    m_pub[id].time_span = std::chrono::duration_cast<std::chrono::duration<double> >( n -  m_pub[id].last );
    m_pub[id].last  = n;
    if( m_pub[id].time_span.count() > m_watchdog )
    {
      return false;
    }
    return true;
}

template< class T> template<class M , class K >
void Controller< T >::add_subscriber(const std::string& id, const std::string &topic, uint32_t queue_size, void(K::*fp)(M), K *obj, const ros::TransportHints &transport_hints)
{
  m_sub[id].sub = m_controller_nh.subscribe<M,K>(topic, queue_size, fp, obj, transport_hints );
  m_sub[id].start = nullptr;
}

template< class T >
bool Controller< T >::tick( const std::string& id )
{
    auto n = std::chrono::high_resolution_clock::now();
    if( m_sub[id].start == nullptr )
    {
      m_sub[id].start = new std::chrono::high_resolution_clock::time_point();
      *m_sub[id].start = n;
      m_sub[id].last  = n;
    }
    m_sub[id].time_span =std::chrono::duration_cast< std::chrono::duration<double> >( n -  m_sub[id].last );
    m_sub[id].last  = n;

    return (m_sub[id].time_span.count() < m_watchdog);
}

template< class T >
ros::Subscriber& Controller< T >::getSubscriber  ( const std::string& id )
{
  return m_sub.at( id ).sub;
}

template< class T >
ros::Publisher&  Controller< T >::getPublisher   ( const std::string& id )
{
  return m_pub.at( id ).pub;
}

template< class T >
bool Controller< T >::callAvailable( const double watchdog )
{
  ros::WallTime st = ros::WallTime::now();

  m_controller_nh_callback_queue.callAvailable( );

  if( m_sub.size() > 0 )
  {
      for( const auto & sub : m_sub )
      {
        if( sub.second.start == nullptr )
        {
          CNR_ERROR_THROTTLE(*m_logger, 5.0, "The topic '"+ sub.second.sub.getTopic() + "' seems not yet published..");
        }
        if( sub.second.time_span.count()> m_watchdog )
        {
          CNR_ERROR_THROTTLE(*m_logger, 5.0, "Watchdog on subscribed topic '"+ sub.second.sub.getTopic()+"' time span: " + std::to_string(sub.second.time_span.count()) + " watchdog: " + std::to_string( m_watchdog ));
        }
      }
  }
  return true;
}

}

#endif
