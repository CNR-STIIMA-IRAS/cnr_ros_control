#ifndef __cnr_configuration_manager_xmlrpc__
#define __cnr_configuration_manager_xmlrpc__

#include <ros/ros.h>
#include <string>

#include <configuration_msgs/ConfigurationComponent.h>
#include <nodelet/NodeletLoadRequest.h>

namespace cnr_configuration_manager
{

namespace param
{



inline bool get_configuration_component_no_dependencies( XmlRpc::XmlRpcValue& configuration_component, ConfigurationStruct& configuration, std::string& error )
{
  if( configuration_component.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    error = "The element is not a struct" ;
    return false;
  }
  if( !configuration_component.hasMember("name") )
  {
    error = "The element has not the field 'name'";
    return false;
  }
  if( !configuration_component.hasMember( "components" ) )
  {
    error = "The element has not the field 'components'";
    return false;
  }
  if( configuration_component["components"].getType() != XmlRpc::XmlRpcValue::TypeArray )
  {
    error = "The element has 'components' bad-formed";
    return false;
  }

  configuration.data.name   = (std::string)configuration_component["name"];
  configuration.data.hidden = ( configuration_component.hasMember( "hidden" ) ) ? (bool)configuration_component["hidden"] : false;
  configuration.data.state  = "unknown";
  configuration.data.type   = "unknown";
  configuration.components.clear();

  size_t jdx = 0;
  for( jdx=0; jdx<configuration_component["components"].size();jdx++)
  {
    XmlRpc::XmlRpcValue& c = configuration_component["components"][jdx];
    if(  c.getType() != XmlRpc::XmlRpcValue::TypeStruct)                 { error = "Broken at component n. " + std::to_string(jdx) + ". Struct broken"; return false; }
    if( !c.hasMember("controller") )                                     { error = "Broken at component n. " + std::to_string(jdx) + ". No 'controller' field"; return false; }
    if( !c.hasMember("hardware_interface") && !c.hasMember("robot_hw") ) { error = "Broken at component n. " + std::to_string(jdx) + ". No 'hardware_interface' field"; return false; }

    std::string key = (std::string)c[ c.hasMember("hardware_interface") ? "hardware_interface" : "robot_hw" ];
    configuration.components[ key ].push_back( (std::string)c["controller"]);
  }

  return true;

}

inline bool get_configuration_components ( XmlRpc::XmlRpcValue&                           configuration_components
                                         , std::map< std::string, ConfigurationStruct >&  configurations
                                         , std::string& error )
{

  if ( configuration_components.getType() != XmlRpc::XmlRpcValue::TypeArray )
  {
    error = "The parameter is not an array.";
    return false;
  }

  configurations.clear();
  std::map<std::string, size_t> name_to_index;
  for(size_t i=0; i < configuration_components.size(); i++)
  {
    std::string         what;
    ConfigurationStruct configuration;
    if( !get_configuration_component_no_dependencies( configuration_components[i], configuration, what ) )
    {
      error = "Error in the configuration " + std::to_string( i ) + ": " +   what;
      return false;
    }

    if( configurations.find( configuration.data.name ) != configurations.end() )
    {
      error = "The configuration '" + configuration.data.name + "' is repeated! Skip the override...";
      configurations.clear();
      return false;
    }

    name_to_index [ configuration.data.name ] = i;
    configurations[ configuration.data.name ] = configuration;
  }



  // ============================================================
  // DEPENDS
  //
  // ============================================================
  for(size_t i=0; i < configuration_components.size(); i++)
  {
    XmlRpc::XmlRpcValue& configuration_component = configuration_components[i];
    if( !configuration_component.hasMember( "depends" ) )
    {
      continue;
    }

    if( configuration_component["depends"].getType() != XmlRpc::XmlRpcValue::TypeArray )
    {
      error = "The element #" + std::to_string( i ) + " has 'depends' bad-formed, while an array was expected";
      return false;
    }

    std::string configuration_name = configuration_component["name"];
    for( size_t j=0; i < configuration_component["depends"].size();j++)
    {
      std::string dep_name = (std::string)configuration_component["depends"][j];
      if( name_to_index.find( dep_name) == name_to_index.end() )
      {
        error = "There is a broken dependency.";
        configurations.clear();
        return false;
      }
      size_t              dep_index = name_to_index[ dep_name ];
      std::string         what;
      ConfigurationStruct configuration_depend_from;
      if( !param::get_configuration_component_no_dependencies( configuration_components[ dep_index ], configuration_depend_from, what ) )
      {
        error = "Error in the configuration " + std::to_string( i ) + ": " + what;
        configurations.clear();
        return false;
      }
      concat( configurations[ configuration_name ].components, configuration_depend_from.components );
    }
  }
  // ============================================================

  return true;
}

}

}

#endif
