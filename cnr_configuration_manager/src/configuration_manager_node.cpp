#include <atomic>
#include <mutex>

#include <ros/ros.h>

#include <nodelet/loader.h>
#include <nodelet/loader.h>
#include <nodelet/NodeletList.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>

#include <cnr_logger/cnr_logger.h>

#include <controller_manager_msgs/ControllerState.h>

#include <configuration_msgs/ConfigurationComponent.h>
#include <configuration_msgs/StartConfiguration.h>
#include <configuration_msgs/StopConfiguration.h>
#include <configuration_msgs/ListConfigurations.h>
#include <configuration_msgs/UpdateConfigurations.h>

#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_configuration_manager/signal_handler.h>
#include <cnr_configuration_manager/internal/cnr_configuration_manager_utils.h>
#include <cnr_configuration_manager/internal/cnr_configuration_manager_xmlrpc.h>
#include <cnr_hardware_nodelet_interface/cnr_hardware_nodelet_interface.h>


namespace cnr_configuration_manager
{

constexpr auto NODELET_CONFIGURATION_MANAGER_NS = "/configuration_manager";

class ConfigurationManager
{
public:

  ConfigurationManager( std::shared_ptr<cnr_logger::TraceLogger>& logger
                      , const ros::NodeHandle& nh
                      , const std::string& configuration_nodelet_manager_ns = NODELET_CONFIGURATION_MANAGER_NS ) noexcept(false)
    : m_nh( nh )
    , m_active_configuration_name( "None" )
    , m_logger( logger )
    , m_hw_nodelet( m_logger, nh.getNamespace(), configuration_nodelet_manager_ns )
  {

  }

  ~ConfigurationManager() noexcept(false)
  {
    try
    {
      CNR_TRACE_START((*m_logger));

      std::string error;
      std::vector<std::string>  hw_names_from_nodelet;
      if( !m_hw_nodelet.list_hw(hw_names_from_nodelet, ros::Duration(10) ) )
      {
        CNR_EXIT_EX((*m_logger), false, "Error in getting the loaded hardware interfaces by the nodelet manager: " + m_hw_nodelet.error() );
      }

      CNR_DEBUG( (*m_logger), "Unload all hw nodelet (" << to_string( hw_names_from_nodelet ) << ")" );
      if(!m_hw_nodelet.unload_hw(hw_names_from_nodelet, ros::Duration(10)) )
      {
        CNR_FATAL((*m_logger), "Unload the configuration failed. Error: " + m_hw_nodelet.error() );
      }

      CNR_INFO((*m_logger),"[ DONE]");
      m_logger.reset();
    }
    catch(std::exception& e)
    {
      std::cerr << "Error in destructor of the ConfigurationManager: " << e.what() << std::endl;
    }
    catch(...)
    {
      std::cerr << "Error in destructor of the ConfigurationManager: Unhandled exception"<< std::endl;
    }
  }

  bool startCallback       ( configuration_msgs::StartConfiguration::Request&   req, configuration_msgs::StartConfiguration::Response&   res)
  {
    CNR_TRACE_START((*m_logger));
    CNR_INFO((*m_logger),"*************************** ******************************** ********************************" );
    CNR_INFO((*m_logger),"*************************** Start Configuration: '"<< req.start_configuration << "'" );
    CNR_INFO((*m_logger),"*************************** ******************************** ********************************" );

    try
    {
      const std::lock_guard<std::mutex> lock(m_callback_mutex);
      if( m_configurations.find(req.start_configuration ) == m_configurations.end() )
      {
        res.ok = false;
        CNR_ERROR((*m_logger), "The Configuration '"+ req.start_configuration + "' is not among the listed.");
      }
      else
      {
        ConfigurationStruct next_configuration  = m_configurations.at( req.start_configuration );
        res.ok = callback( &next_configuration, req.strictness,  ros::Duration(10.0) );
        if( res.ok )
        {
          m_active_configuration      = next_configuration;
          m_active_configuration_name = req.start_configuration;
          m_nh.setParam("status/active_configuration", m_active_configuration_name);
        }
      }
    }
    catch( std::exception& e)
    {
      CNR_RETURN_FALSE((*m_logger), "Exception in start configuration: " + std::string( e.what() ));
    }
    catch( ... )
    {
      CNR_RETURN_FALSE((*m_logger), "Exception in start configuration. Service Failure.");
    }
    CNR_RETURN_TRUE((*m_logger));
  }

  bool stopCallback        ( configuration_msgs::StopConfiguration::Request&    req, configuration_msgs::StopConfiguration::Response&    res)
  {
    CNR_TRACE_START((*m_logger));
    CNR_INFO((*m_logger), "*************************** ******************************** ********************************" );
    CNR_INFO((*m_logger), "*************************** Stop Configuration" );
    CNR_INFO((*m_logger), "*************************** ******************************** ********************************" );

    try
    {
      const std::lock_guard<std::mutex> lock(m_callback_mutex);
      res.ok = callback(nullptr, req.strictness, ros::Duration(10.0));
      if( res.ok )
      {
        m_active_configuration      = ConfigurationStruct();
        m_active_configuration_name = "";
        m_nh.setParam("status/active_configuration", m_active_configuration_name);
      }
    }
    catch( std::exception& e)
    {
      CNR_RETURN_FALSE((*m_logger), "Exception in stop configuration: " + std::string( e.what() ));
    }
    catch( ... )
    {
      CNR_RETURN_FALSE((*m_logger), "Exception in stop configuration. Service Failure.");
    }
    CNR_RETURN_TRUE((*m_logger));
  }

  bool listConfigurations  ( configuration_msgs::ListConfigurations::Request&   req, configuration_msgs::ListConfigurations::Response&   res)
  {
    CNR_TRACE_START((*m_logger));
    res.configurations.clear();
    const std::lock_guard<std::mutex> lock(m_callback_mutex);
    if( !updateConfigurations( ) )
    {
      CNR_RETURN_FALSE((*m_logger), "Update COnfiguration Failed.");
    }

    res.configurations.resize( m_configurations.size() );
    for( auto it = m_configurations.begin(); it != m_configurations.end(); it++)
    {
      size_t i = std::distance( m_configurations.begin(), it );
      if(!cast( it->second, res.configurations.at(i)  ) )
      {
        CNR_RETURN_BOOL((*m_logger), false, "Cast Error");
      }
    }
    CNR_RETURN_TRUE((*m_logger));
  }

  bool updateConfigurations( configuration_msgs::UpdateConfigurations::Request& req, configuration_msgs::UpdateConfigurations::Response& res)
  {
    CNR_TRACE_START((*m_logger));
    const std::lock_guard<std::mutex> lock(m_callback_mutex);
    bool ret = updateConfigurations( );
    CNR_RETURN_BOOL((*m_logger), ret);
  }


  bool init ( )
  {
    CNR_TRACE_START(*m_logger);
    try
    {

      CNR_WARN(*m_logger, "********************* INIT ****************************");
      m_load_configuration      = m_nh.advertiseService("start_configuration", &cnr_configuration_manager::ConfigurationManager::startCallback,      this );
      m_unload_configuration    = m_nh.advertiseService("stop_configuration" , &cnr_configuration_manager::ConfigurationManager::stopCallback,       this );
      m_list_controller_service = m_nh.advertiseService("list_configurations", &cnr_configuration_manager::ConfigurationManager::listConfigurations, this );

      CNR_TRACE_START( (*m_logger) );
      if( m_nh.hasParam( "control_configurations" ) && updateConfigurations( )  )
      {
        m_nh.setParam("status/active_configuration", "none");
      }
      else
      {
        CNR_RETURN_FALSE(*m_logger, "Configuration Manager: param '" + m_nh.getNamespace() + "/control_configurations' is broken. Abort" );
      }

      m_signal_handler.setupSignalHandlers();

    }
    catch (SignalException& e)
    {
      CNR_RETURN_FALSE(*m_logger, "Error in the initialization. Abort.");
    }
    CNR_RETURN_TRUE(*m_logger);
  }


  bool run  ( )
  {
    CNR_TRACE_START(*m_logger);
    try
    {
      const int decimator = 100;
      ros::Rate lp(decimator);
      size_t cnt = 0;
      CNR_WARN(*m_logger, "********************* RUN ****************************");
      while ( ros::ok() )
      {
        bool full_check = ( ( (cnt++ ) % decimator ) == 0 );
        if ( !isOk( full_check ) )
        {
          CNR_WARN_THROTTLE(*m_logger, 2,"Raised an Error by one of the Hw. Try to Stop configuration because a error has been raised by one of the Hw ");
          configuration_msgs::StopConfiguration srv;
          srv.request.strictness=1;
          if (!stopCallback(srv.request,srv.response))
          {
            CNR_WARN_THROTTLE(*m_logger, 2,"Impossible to Stop the Running Configuration. Broken Service.");
            return false;
          }
          if (!srv.response.ok)
          {
            CNR_WARN_THROTTLE(*m_logger, 2,"Impossible to Stop the Running Configuration. The service response is negative.");
            return false;
          }
        }
        else
        {
          CNR_WARN_THROTTLE(*m_logger, 10.0,"Waiting for a new callback, or some change in the cofiguration state");
        }

        if( m_signal_handler.gotExitSignal() )
        {
          CNR_WARN_THROTTLE(*m_logger, 2,"SIGINT called. Controlled exit.");
          configuration_msgs::StopConfiguration stop;
          stop.request.strictness = 1;
          if(!stopCallback(stop.request,stop.response))
          {
              CNR_FATAL(*m_logger, "Error in stopping the configuration ....");
          }
          break;    // exit normally after SIGINT
        }
        lp.sleep();
      }
    }
    catch( std::exception& e )
    {
      std::cerr << "Configuration Manager - Exception: " << e.what() << std::endl;
    }
    CNR_RETURN_TRUE(*m_logger);
  }

  bool isOk ( bool nodelet_check )
  {
    std::string error;
    try
    {
      for( auto const & component : m_active_configuration.components )
      {
        const std::string& hw = component.first;
        cnr_hardware_interface::StatusHw hw_status;

        if( !cnr_hardware_interface::get_state(m_nh, hw, hw_status, ros::Duration(0.01), error ) )
        {
          CNR_FATAL((*m_logger), "The HW " << hw << " has not any valid state: " << error);
          return false;
        }

        if( ( hw_status == cnr_hardware_interface::ERROR  ) || ( hw_status == cnr_hardware_interface::CTRL_ERROR) || ( hw_status == cnr_hardware_interface::SRV_ERROR) )
        {
          CNR_FATAL((*m_logger), "The status of the HW '" << hw << "' is " << cnr_hardware_interface::to_string( hw_status) );
          return false;
        }
        for( auto const & ctrl : component.second )
        {
          std::string ctrl_status;
          if( !cnr_controller_interface::get_state(hw, ctrl, ctrl_status, error, ros::Duration(0.01) ) )
          {
            CNR_FATAL((*m_logger), "The HW '" << hw << "' and CTRL '" << ctrl << "' has not any valid state: " << error);
            return false;
          }
          if( ctrl_status == "ERROR" )
          {
            CNR_FATAL((*m_logger),"The status of HW '" << hw << "' and CTRL '" << ctrl << "' is " << ctrl_status << " while it should be 'RUNNING'");
            return false;
          }
        }
      }

      if( nodelet_check )
      {
        std::vector<std::string> hw_names_from_nodelet;
        if(!m_hw_nodelet.list_hw(hw_names_from_nodelet, ros::Duration(0.1)) )
        {
          CNR_FATAL((*m_logger), "HW Nodelet Manager failed: " << m_hw_nodelet.error() );
          return false;
        }

        for( auto const & component : m_active_configuration.components )
        {
          const std::string& hw = component.first;
          if( std::find( hw_names_from_nodelet.begin(), hw_names_from_nodelet.end(), hw ) == hw_names_from_nodelet.end() )
          {
            CNR_FATAL((*m_logger), "HW " << hw << " seems not loaded in memory!" );
            return false;
          }
          std::vector< controller_manager_msgs::ControllerState >  running;
          std::vector< controller_manager_msgs::ControllerState >  stopped;
          std::string error;
          if( !m_hw_nodelet.cmi_.at(hw).list_ctrl(running, stopped, ros::Duration(1.0) ) )
          {
            CNR_FATAL((*m_logger), "Ctrl of the HW" << hw << " seems not working properly: " << m_hw_nodelet.cmi_.at(hw).error() );
            return false;
          }
          for( auto const & ctrl : component.second )
          {
            if( std::find_if( running.begin(), running.end(), [&ctrl]( auto r){ return r.name == ctrl; }) == running.end() )
            {
              CNR_WARN((*m_logger), "CTRL " << ctrl << " of the HW" << hw << " is not running! ");
              return false;
            }
          }
        }
      }
    }
    catch(std::exception& e)
    {
      CNR_RETURN_FATAL(*m_logger, "Exception while checking the status of the Hw. Exception: " + std::string( e.what() ) );
    }
    catch( ... )
    {
      CNR_FATAL(*m_logger, "Unhandled Exception while checking the status of the Hw. " );
      return false;
    }
    return true;
  }

private:
  ros::NodeHandle                                  m_nh;
  std::shared_ptr<cnr_logger::TraceLogger>         m_logger;
  std::mutex                                       m_callback_mutex;
  std::string                                      m_active_configuration_name;
  ConfigurationStruct                              m_active_configuration;
  std::map<std::string, ConfigurationStruct>       m_configurations;

  cnr_hardware_nodelet_interface::NodeletManagerInterface  m_hw_nodelet;

  ros::ServiceServer                               m_load_configuration      ;
  ros::ServiceServer                               m_unload_configuration    ;
  ros::ServiceServer                               m_list_controller_service ;

  SignalHandler                                    m_signal_handler;




  bool checkRobotHwState( const std::string& hw, cnr_hardware_interface::StatusHw target = cnr_hardware_interface::RUNNING )
  {
    cnr_hardware_interface::StatusHw hw_status;
    ros::NodeHandle n("/");
    std::string error;
    if( !cnr_hardware_interface::get_state(n, hw, hw_status, ros::Duration(10), error ) )
    {
      CNR_FATAL((*m_logger), "The HW " << hw << " has not any valid state: " << error);
      return false;
    }
    if( hw_status != target )
    {
      CNR_FATAL((*m_logger), "The status of the HW '" << hw << "' is " << cnr_hardware_interface::to_string( hw_status) << " while it should be " << cnr_hardware_interface::to_string( target ));
      return false;
    }
    return true;
  }
  bool callback               (ConfigurationStruct* next_configuration, const int &strictness, const ros::Duration& watchdog  )
  {
    CNR_TRACE_START((*m_logger));
    std::string error;

    const std::vector<std::string>  hw_active_names = getHardwareInterfacesNames( m_active_configuration );
    const std::vector<std::string>  hw_next_names   = next_configuration ?  getHardwareInterfacesNames( *next_configuration ) : std::vector<std::string>();
    std::vector<std::string>        hw_to_load_names;
    std::vector<std::string>        hw_to_unload_names;
    std::vector<std::string>        hw_names_from_nodelet;


    CNR_INFO( (*m_logger), cnr_logger::BOLDMAGENTA() << ">>>>>>>>>>>> Configuring HW " << cnr_logger::RESET());
    if( !m_hw_nodelet.list_hw(hw_names_from_nodelet, watchdog) )
    {
      CNR_RETURN_FALSE((*m_logger), "Error in getting the loaded hardware interfaces by the nodelet manager: " + m_hw_nodelet.error() );
    }

    extract<std::string>(hw_next_names, hw_active_names, &hw_to_load_names, &hw_to_unload_names, nullptr );
    CNR_DEBUG((*m_logger), "HW NAMES - ACTIVE (CLASS)  : " << to_string(hw_active_names      ));
    CNR_DEBUG((*m_logger), "HW NAMES - ACTIVE (NODELET): " << to_string(hw_names_from_nodelet));
    CNR_DEBUG((*m_logger), "HW NAMES - NEXT            : " << to_string(hw_next_names         ));
    CNR_DEBUG((*m_logger), "HW NAMES - TO LOAD         : " << to_string(hw_to_load_names      ));
    CNR_DEBUG((*m_logger), "HW NAMES - TO UNLOAD       : " << to_string(hw_to_unload_names    ));

    CNR_INFO( (*m_logger), "Check coherence between nodelet status and configuration manager status");
    if( !equal(hw_active_names, hw_names_from_nodelet ))
    {
      CNR_WARN((*m_logger), "Active configuration and the nodelet status is different. We force the unload of all the nodelet.. cross the fingers");
      if( !m_hw_nodelet.purge_hw( watchdog) )
      {
        CNR_RETURN_FALSE((*m_logger), "The purge of the nodelets failed: " + m_hw_nodelet.error());
      }
      hw_to_load_names.insert(hw_to_load_names.begin(),hw_active_names.begin(), hw_active_names.end());
    }

    CNR_INFO((*m_logger), "Load the needed hardware interfaces by nodelets:" << to_string(hw_to_load_names, "") );
    for( const auto & hw_to_load_name : hw_to_load_names)
    {
      if(!m_hw_nodelet.load_hw( hw_to_load_name, watchdog, true) )
      {
        CNR_RETURN_FALSE((*m_logger), "Loading of the RobotHW '"+ hw_to_load_name +"' failed. Error:\n\t=>" + m_hw_nodelet.error());
      }

      if( !checkRobotHwState( hw_to_load_name ) )
      {
        CNR_RETURN_FALSE((*m_logger));
      }
    }
    CNR_INFO( (*m_logger), cnr_logger::BOLDMAGENTA() << "<<<<<<<<<<<< Configuring HW " << cnr_logger::RESET() );


    CNR_INFO( (*m_logger), cnr_logger::BOLDMAGENTA() << ">>>>>>>>>>>> Controllers LifeCycle Management doswitch (hw: "
              << cnr_controller_interface::to_string( hw_next_names ) << ")"  << cnr_logger::RESET());
    for( auto const & hw_name : hw_next_names )
    {
      // create proper configured servers
      if( m_hw_nodelet.cmi_.find(hw_name) == m_hw_nodelet.cmi_.end() )
      {
        m_hw_nodelet.cmi_.emplace(hw_name, cnr_controller_manager_interface::ControllerManagerInterface( m_logger, hw_name) );
      }

      std::vector<std::string>* next_controllers = (next_configuration ? &(next_configuration->components.at(hw_name) ) : nullptr );
      CNR_DEBUG((*m_logger), "List of controllers to be uploaded for the RobotHw: " << hw_name << " next controllers: " << (next_controllers ?  to_string( *next_controllers ) : "none" ) )  ;
      if( !m_hw_nodelet.cmi_.at(hw_name).doswitch(strictness, next_controllers, ros::Duration(10.0)  ))
      {
          CNR_RETURN_FALSE((*m_logger), "Error in switching the controller");
      }
    }
    CNR_INFO( (*m_logger), cnr_logger::BOLDMAGENTA() << "<<<<<<<<<<<< Controllers LifeCycle Management doswitch " << cnr_logger::RESET() );


    CNR_INFO( (*m_logger), cnr_logger::BOLDMAGENTA() << ">>>>>>>>>>>> Controllers LifeCycle Management unload (hw: " << cnr_controller_interface::to_string( hw_next_names ) << ")" << cnr_logger::RESET() );
    if( !cnr_controller_manager_interface::stop_unload_all_ctrl( m_hw_nodelet.cmi_, hw_to_unload_names, ros::Duration(10.0) )  )
    {
      CNR_INFO( (*m_logger), cnr_logger::BOLDMAGENTA() << "<<<<<<<<<<<< Controllers LifeCycle Management unload " << cnr_logger::RED() << "FAILED" << cnr_logger::RESET() );
      CNR_RETURN_FALSE(*m_logger);
    }


    CNR_INFO( (*m_logger),  cnr_logger::BOLDMAGENTA() <<  ">>>>>>>>>>>> Unload unnecessary hw (" <<to_string( hw_to_unload_names ) << ")" << cnr_logger::RESET() );
    if(!m_hw_nodelet.unload_hw(hw_to_unload_names, watchdog) )
    {
      CNR_RETURN_FALSE((*m_logger), "Unload the configuration failed. Error: " + m_hw_nodelet.error() );
    }
    CNR_INFO( (*m_logger),  cnr_logger::BOLDMAGENTA() <<  "<<<<<<<<<<<< Unload unnecessary hw [DONE]"<< cnr_logger::RESET() );


    CNR_DEBUG( (*m_logger), "It is finished.");
    CNR_RETURN_TRUE((*m_logger));
  }

  bool updateConfigurations   ( )
  {
    CNR_TRACE_START((*m_logger));

    std::map<std::string, ConfigurationStruct > configurations;
    XmlRpc::XmlRpcValue                         configuration_components;
    if(!m_nh.getParam("control_configurations", configuration_components ) )
    {
      std::string error = "Param '" + m_nh.getNamespace() + "/control_configurations' is not found." ;
      CNR_RETURN_BOOL((*m_logger), false, error);
    }

    std::string error;
    if(!param::get_configuration_components( configuration_components, configurations, error ) )
    {
      error = "Param '" + m_nh.getNamespace() + "/control_configurations' error: " + m_hw_nodelet.error() ;
      CNR_RETURN_BOOL((*m_logger), false, error);
    }

    m_configurations.clear();
    m_configurations = configurations;
    CNR_RETURN_BOOL((*m_logger), true);
  }

};
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "cnr_configuration_manager", ros::init_options::NoSigintHandler );
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(4);

  spinner.start();

  try
  {
    std::string n = ros::this_node::getName(); n.erase(0,1);
    std::replace( n.begin(), n.end(), '/', '_');

    std::shared_ptr<cnr_logger::TraceLogger>                         logger( new cnr_logger::TraceLogger( n, nh.getNamespace(), true ) );
    std::shared_ptr<cnr_configuration_manager::ConfigurationManager> cm    ( new cnr_configuration_manager::ConfigurationManager(logger, nh) );

    if(!cm->init())
    {
      CNR_FATAL( *logger, "Error in ConfigurationManager init. Exit.");
      return -1;
    }

    if( !cm->run() )
    {
      CNR_FATAL( *logger, "Error in ConfigurationManager execution. Exit.");
      return -1;
    }

    CNR_INFO( *logger, "Destroy the ConfigurationManager.");
    cm.reset();

    CNR_INFO( *logger, "Destroy the Logger.");
    logger.reset();

  }
  catch( std::exception& e )
  {
    std::cerr << "Error in Configuration Manager. Exception: " << e.what() << std::endl;
  }



  return 0;
}
