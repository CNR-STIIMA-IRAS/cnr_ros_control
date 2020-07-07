#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_hardware_interface/cnr_robot_hw.h>

#define GENERATE_ENUM_STRINGS  // Start string generation
#include <cnr_hardware_interface/internal/cnr_robot_hw_status.h>
#undef GENERATE_ENUM_STRINGS   // Stop string generation

namespace cnr_hardware_interface
{

RobotHW::RobotHW()
  : m_status (cnr_hardware_interface::CREATED), m_shutted_down( false ), m_is_first_read(true), m_set_param(nullptr)
{
//  if ( m_resource_names.size() == 0 )
//  {
//    dump_state( cnr_hardware_interface::ERROR );
//  }
  dump_state( cnr_hardware_interface::CREATED );
}

RobotHW::~RobotHW()
{
  CNR_TRACE_START(*m_logger);
  if(!m_shutted_down)
  {
    if(!shutdown() )
    {
      dump_state( cnr_hardware_interface::ERROR );
    }
    else
    {
      dump_state( cnr_hardware_interface::SHUTDOWN );
    }
  }
  CNR_TRACE( *m_logger, "[  DONE] ");
}

bool RobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{

  std::string n = "HW" + robot_hw_nh.getNamespace();
  std::replace( n.begin(), n.end(), '/', '_');

  m_logger.reset( new cnr_logger::TraceLogger( n, robot_hw_nh.getNamespace() ) );
  CNR_TRACE_START(*m_logger);
  if( enterInit( root_nh, robot_hw_nh )  && doInit( ) && exitInit ( ) )
  {
    CNR_RETURN_TRUE( *m_logger, "RobotHW '" + m_robot_name + "' Initialization OK");
  }


  CNR_RETURN_FALSE( *m_logger,  "RobotHW '" + m_robot_name + "' Initialization Failed" );
}

void RobotHW::read(const ros::Time& time, const ros::Duration& period) 
{
  CNR_TRACE_START_THROTTLE(*m_logger, 5.0 );

  m_robot_hw_queue.callAvailable();

  if( !doRead( time, period ) )
  {
    dump_state( cnr_hardware_interface::ERROR );
    CNR_RETURN_NOTOK_THROTTLE( *m_logger, void(), 5.0 );
  }

  if( m_is_first_read )
  {
    dump_state( cnr_hardware_interface::RUNNING );
    m_is_first_read = false;
    if( m_set_param )
    {
      m_set_param("first_configuration");
    }
  }

  CNR_RETURN_OK_THROTTLE( *m_logger, void(), 5.0 );
}

void RobotHW::write(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE(*m_logger, 5.0 );
  if( enterWrite(  ) && doWrite(time, period) && exitWrite( ) )
  {
    CNR_RETURN_OK_THROTTLE( *m_logger, void(), 5.0 );
  }

  dump_state( cnr_hardware_interface::ERROR );
  CNR_RETURN_NOTOK_THROTTLE( *m_logger, void(), 5.0, "Error in writing..." );
}

bool RobotHW::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  CNR_TRACE_START(*m_logger );
  if( enterPrepareSwitch( start_list, stop_list ) && doPrepareSwitch(start_list, stop_list) && exitPrepareSwitch( ) )
  {
    CNR_RETURN_TRUE( *m_logger );
  }

  m_is_first_read = false;
  CNR_RETURN_FALSE( *m_logger );
}

bool RobotHW::checkForConflict(const std::list< hardware_interface::ControllerInfo >& info)
{
  CNR_TRACE_START(*m_logger );
  if( enterCheckForConflict( info )  && doCheckForConflict( info ) && exitCheckForConflict( ) )
  {
    CNR_RETURN_TRUE( *m_logger );
  }

  CNR_RETURN_FALSE( *m_logger );
}

bool RobotHW::shutdown() 
{
  CNR_TRACE_START(*m_logger, ">>>> RobotHW Shutdown (" + m_robot_name + ")" );
  if( enterShutdown(  )  && doShutdown( ) && exitShutdown ( ) )
  {
    CNR_RETURN_TRUE( *m_logger, "<<<< Robot Shutdown (" + m_robot_name + ")" );
  }
  dump_state(cnr_hardware_interface::ERROR);
  CNR_RETURN_FALSE( *m_logger, "<<<< Robot Shutdown Failure (" + m_robot_name + ")" );
}

bool RobotHW::enterShutdown()
{
  CNR_TRACE_START(*m_logger);
  CNR_RETURN_TRUE(*m_logger);
}

bool RobotHW::exitShutdown()
{
  CNR_TRACE_START(*m_logger);
  if( m_set_param )
  {
    m_set_param("shutdown_configuration");
  }
  m_shutted_down =  true;
  bool ret = dump_state( cnr_hardware_interface::SHUTDOWN );
  CNR_RETURN_BOOL(*m_logger, ret);
}

bool RobotHW::enterWrite()
{
  return true;
}

bool RobotHW::exitWrite()
{
  if( m_prev_status != m_status )
  {
    if( m_set_param )
    {
      m_set_param("last_valid_configuration");
    }
    m_prev_status = m_status;
  }

  return dump_state();
}

bool RobotHW::enterInit(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  CNR_TRACE_START(*m_logger);
  m_robot_hw_nh = robot_hw_nh;
  m_robot_hw_nh.setCallbackQueue(&m_robot_hw_queue);
  m_root_nh     = root_nh;
  m_stop_thread = false;
  m_robot_name  = extractRobotName( m_robot_hw_nh.getNamespace() );
  m_set_params_server = m_robot_hw_nh.advertiseService("writeParams",&cnr_hardware_interface::RobotHW::setParamServer,this);
  m_get_params_server = m_robot_hw_nh.advertiseService("readParams" ,&cnr_hardware_interface::RobotHW::getParamServer,this);

  m_robot_hw_queue.callAvailable();

  CNR_RETURN_TRUE(*m_logger);
}

bool RobotHW::exitInit()
{
  CNR_TRACE_START(*m_logger);
  bool ret = ( m_resource_names.size() > 0 );

  if( !ret )
  {
    CNR_FATAL(*m_logger, "Reources names not set! Remeber to assign them in the doInit function. ");
  }

  dump_state( ret ? cnr_hardware_interface::INITIALIZED : cnr_hardware_interface::ERROR );
  CNR_RETURN_BOOL(*m_logger, ret);
}

bool RobotHW::enterPrepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  CNR_TRACE_START(*m_logger);
  for (const hardware_interface::ControllerInfo& ctrl: stop_list)
  {
    bool not_prensent=true;
    std::list<hardware_interface::ControllerInfo>::iterator stopped_controller;
    for (std::list<hardware_interface::ControllerInfo>::iterator it=m_active_controllers.begin();it!=m_active_controllers.end();++it)
    {
      if (!it->name.compare(ctrl.name))
      {
        stopped_controller=it;
        not_prensent=false;
        break;
      }
    }
    if (not_prensent)
    {
      add_diagnostic_message( "ERROR", "controller '" + ctrl.name + "' is not active, so I cannot stop it", { {"Transition", "switching"} }, true);
      dump_state( cnr_hardware_interface::CTRL_ERROR );
      CNR_RETURN_FALSE(*m_logger);
    }
    m_active_controllers.erase(stopped_controller);
  }
  for (const hardware_interface::ControllerInfo& ctrl: start_list)
  {
    bool already_prensent=false;
    std::list<hardware_interface::ControllerInfo>::iterator stopped_controller;
    for (std::list<hardware_interface::ControllerInfo>::iterator it=m_active_controllers.begin();it!=m_active_controllers.end();it++)
    {
      if (!it->name.compare(ctrl.name))
      {
        stopped_controller=it;
        already_prensent=true;
        break;
      }
    }
    if (already_prensent)
    {
      CNR_WARN(*m_logger, "controller "<< ctrl.name << "is not active, so I cannot stop it");
    }
    else
    {
      m_active_controllers.push_back(ctrl);
    }
  }
  CNR_RETURN_TRUE(*m_logger);
}

bool RobotHW::exitPrepareSwitch( )
{
  return dump_state( cnr_hardware_interface::READY );
}

bool RobotHW::enterCheckForConflict(const std::list< hardware_interface::ControllerInfo >& info)
{
  if(! hardware_interface::RobotHW::checkForConflict(info) )
  {
    CNR_FATAL(*m_logger, "Base Class RobotHw failed in the conflict checking!");
    return true;
  }

  // Each controller can use more than a hardware_interface for a single joint (for example: position, velocity, effort).
  // One controller can control more than one joint.
  // A joint can be used only by a controller.
  std::vector<bool> global_resource_used( m_resource_names.size() );
  std::fill(global_resource_used.begin(),global_resource_used.end(),false);

  for (hardware_interface::ControllerInfo controller: info)
  {
    std::vector<bool> single_controller_resource_used(m_resource_names.size());
    std::fill(single_controller_resource_used.begin(), single_controller_resource_used.end(),false);

    for (hardware_interface::InterfaceResources res: controller.claimed_resources)
    {
      for (std::string name: res.resources)
      {
        for (unsigned int iJ=0;iJ<m_resource_names.size();iJ++)
        {
          if (!name.compare(m_resource_names.at(iJ)))
          {
            if (global_resource_used.at(iJ)) // if already used by another
            {
              add_diagnostic_message( "ERROR", "Joint " + name + "%s is already used by another controller", {{"Transition", "switching"}}, true );
              dump_state( cnr_hardware_interface::CTRL_ERROR );
              return true;
            }
            else
              single_controller_resource_used.at(iJ);
          }
        }
      }
    }
    for (unsigned int iJ=0;iJ<m_resource_names.size() ;iJ++)
    {
      global_resource_used.at(iJ)= global_resource_used.at(iJ) || single_controller_resource_used.at(iJ);
    }
  }
  return false;
}

bool RobotHW::getParamServer(configuration_msgs::GetConfigRequest& req, configuration_msgs::GetConfigResponse& res)
{
  return true;
}

bool RobotHW::setParamServer(configuration_msgs::SetConfigRequest& req, configuration_msgs::SetConfigResponse& res)
{
  return true;
}

void RobotHW::add_diagnostic_message  ( const std::string& level
                                      , const std::string& summary
                                      , const std::map<std::string,std::string>& key_values
                                      , const bool verbose)
{
  diagnostic_msgs::DiagnosticStatus diag;
  diag.name        = m_robot_name;
  diag.hardware_id = m_robot_name;
  diag.message     = summary;

  if( level == "OK"   ) { diag.level = diagnostic_msgs::DiagnosticStatus::OK;     CNR_INFO_COND( *m_logger, verbose, "[" << m_robot_name << " ] " << summary );}
  if( level == "WARN" ) { diag.level = diagnostic_msgs::DiagnosticStatus::WARN;   CNR_WARN_COND( *m_logger, verbose, "[" << m_robot_name << " ] " << summary );}
  if( level == "ERROR" ){ diag.level = diagnostic_msgs::DiagnosticStatus::ERROR;  CNR_ERROR_COND(*m_logger, verbose, "[" << m_robot_name << " ] " << summary );}
  if( level == "STALE" ){ diag.level = diagnostic_msgs::DiagnosticStatus::STALE;  CNR_INFO_COND( *m_logger, verbose, "[" << m_robot_name << " ] " << summary );}

  for( const auto & key_value : key_values)
  {
    diagnostic_msgs::KeyValue kv; kv.key = key_value.first; kv.value = key_value.second;
    diag.values.push_back(kv);
  }
  std::lock_guard<std::mutex> lock(m_mutex);
  m_diagnostic.status.push_back(diag);
}


void RobotHW::diagnostics( diagnostic_updater::DiagnosticStatusWrapper &stat )
{
  stat.hardware_id = m_robot_name;
  stat.name        = m_robot_name;

  std::lock_guard<std::mutex> lock(m_mutex);
  if( m_diagnostic.status.size() )
  {
    for( const auto & s : m_diagnostic.status )
    {
      stat.summary(s.level, s.message);
      for( const auto & kv : s.values )
      {
        stat.add( kv.key, kv.value );
      }
    }
    m_diagnostic.status.clear();
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "None Error in the queue");
  }
}


bool RobotHW::dump_state( )
{
  std::string last_status = cnr_hardware_interface::to_string( m_status );
  if( m_status_history.size() == 0 )
  {
    m_status_history.push_back( last_status );
  }
  else
  {
    if( m_status_history.back() != last_status )
    {
      m_status_history.push_back(last_status);
      m_robot_hw_nh.setParam( last_status_param(m_robot_hw_nh.getNamespace()), last_status );
      m_robot_hw_nh.setParam( status_param     (m_robot_hw_nh.getNamespace()), m_status_history );
      CNR_DEBUG( *m_logger, "RobotHW '" << m_robot_hw_nh.getNamespace() <<"' New Status " <<  cnr_hardware_interface::to_string( m_status )  );
    }
  }
  return true;
}

bool RobotHW::dump_state( const cnr_hardware_interface::StatusHw& status )
{
    m_prev_status = m_status;
    m_status = status;
    return dump_state( );
}


}
