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
#ifndef __cnr_configuration_manager_xmlrpc__
#define __cnr_configuration_manager_xmlrpc__

#include <ros/ros.h>
#include <string>

#include <configuration_msgs/ConfigurationComponent.h>
#include <nodelet/NodeletLoadRequest.h>
#include <cnr_configuration_manager/internal/cnr_configuration_manager_utils.h>

namespace cnr_configuration_manager
{

namespace param
{



inline 
bool get_configuration_component_no_dependencies(XmlRpc::XmlRpcValue& configuration_component, 
                                                 cnr_configuration_manager::ConfigurationStruct& configuration,
                                                 std::string& error)
{
  if (configuration_component.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    error = "The element is not a struct" ;
    return false;
  }
  if (!configuration_component.hasMember("name"))
  {
    error = "The element has not the field 'name'";
    return false;
  }
  if (!configuration_component.hasMember("components"))
  {
    error = "The element has not the field 'components'";
    return false;
  }
  if (configuration_component["components"].getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    error = "The element has 'components' bad-formed";
    return false;
  }

  configuration.data.name   = (std::string)configuration_component["name"];
  configuration.data.hidden = (configuration_component.hasMember("hidden")) 
                            ? (bool)configuration_component["hidden"] : false;
  configuration.data.state  = "unknown";
  configuration.data.type   = "unknown";
  configuration.components.clear();

  int jdx = 0;
  for (jdx = 0; jdx < configuration_component["components"].size(); jdx++)
  {
    XmlRpc::XmlRpcValue& c = configuration_component["components"][jdx];
    if (c.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      error = "Broken at component n. " + std::to_string(jdx) + ". Struct broken";
      return false;
    }
    if (!c.hasMember("controller"))
    {
      error = "Broken at component n. " + std::to_string(jdx) + ". No 'controller' field";
      return false;
    }
    if (!c.hasMember("hardware_interface") && !c.hasMember("robot_hw"))
    {
      error = "Broken at component n. " + std::to_string(jdx) + ". No 'hardware_interface' field";
      return false;
    }

    std::string key = (std::string)c[ c.hasMember("hardware_interface") ? "hardware_interface" : "robot_hw" ];
    configuration.components[ key ].push_back((std::string)c["controller"]);
  }

  return true;

}


inline 
bool get_configuration_component_dependencies(XmlRpc::XmlRpcValue&      configuration_component, 
                                              std::vector<std::string>& dependencies, 
                                              std::string&              error)
{
  dependencies.clear();
  if (configuration_component.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    error = "The element is not a struct" ;
    return false;
  }
  if (!configuration_component.hasMember("depends"))
  {
    return true;
  }
  
  for (int jdx = 0; jdx < configuration_component["depends"].size(); jdx++)
  {
    std::string d = configuration_component["depends"][jdx];
    dependencies.push_back(d);
  }

  return true;
}


inline bool get_configuration_components(XmlRpc::XmlRpcValue&                           configuration_components,
                                         std::map< std::string, ConfigurationStruct >&  configurations,
                                         std::string&                                   error)
{

  if (configuration_components.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    error = "The parameter is not an array.";
    return false;
  }

  configurations.clear();
  std::map<std::string, size_t> name_to_index;
  std::map<std::string, std::vector< std::string > > name_to_dep_names;
  for(int i=0; i<configuration_components.size(); i++)
  {
    std::string         what;
    ConfigurationStruct configuration;
    if (!get_configuration_component_no_dependencies(configuration_components[i], configuration, what))
    {
      error = "Error in the configuration " + std::to_string(i) + ": " +   what;
      return false;
    }
    if (configurations.find(configuration.data.name) != configurations.end())
    {
      error = "The configuration '" + configuration.data.name + "' is repeated! Skip the override...";
      configurations.clear();
      return false;
    }
    
    configurations   [ configuration.data.name ] = configuration;
    name_to_index    [ configuration.data.name ] = i;
    name_to_dep_names[ configuration.data.name ] = {};    
    if (!get_configuration_component_dependencies(configuration_components[i], name_to_dep_names[configuration.data.name], what))
    {
      error = "Error in the configuration " + std::to_string(i) + ": " +   what;
      return false;
    }
  }

  // solve all the dependencies
  do
  {
    size_t insertions = 0;
    for (auto & deps : name_to_dep_names )
    {
      std::vector<std::string>& deps_1level = deps.second; 
      for (auto & dep_1level : deps_1level)
      {
        std::vector<std::string> deps_2level = name_to_dep_names[ dep_1level ];
        for ( auto & dep_2level : deps_2level )
        {
          if (std::find(deps_1level.begin(), deps_1level.end(), dep_2level) == deps_1level.end())
          {
            deps_1level.push_back( dep_2level );
            insertions++;
          }
        }
      }
    }
    if(insertions == 0)
    {
      break;
    }
  } while (1);  // solve all the dependencies

  for (auto & configuration : configurations )
  {  
    std::vector<std::string>& dep_names   = name_to_dep_names.at( configuration.first );
    for ( size_t i=0; i<dep_names.size(); i++) 
    {
      if (name_to_index.find(dep_names.at(i)) == name_to_index.end())
      {
        error = "There is a broken dependency.";
        configurations.clear();
        return false;
      }
      std::string         what;
      ConfigurationStruct configuration_depend_from;
      if (!param::get_configuration_component_no_dependencies(
                configuration_components[ name_to_index.at( dep_names.at(i) ) ], configuration_depend_from, what))
      {
        error = "Error in the configuration " + std::to_string(i) + ": " + what;
        configurations.clear();
        return false;
      }
      concat(configuration.second.components, configuration_depend_from.components);      
    }
  }
  // ============================================================

  return true;
}

}

}

#endif


#if 0

 configurations.clear();
  std::map<std::string, size_t> name_to_index;
  for (size_t i = 0; i < configuration_components.size(); i++)
  {
    std::string         what;
    ConfigurationStruct configuration;
    if (!get_configuration_component_no_dependencies(configuration_components[i], configuration, what))
    {
      error = "Error in the configuration " + std::to_string(i) + ": " +   what;
      return false;
    }

    if (configurations.find(configuration.data.name) != configurations.end())
    {
      error = "The configuration '" + configuration.data.name + "' is repeated! Skip the override...";
      configurations.clear();
      return false;
    }
  
    name_to_index [ configuration.data.name ] = i;
    configurations[ configuration.data.name ] = configuration;
  }

for (size_t i = 0; i < configuration_components.size(); i++)
  {
    XmlRpc::XmlRpcValue& configuration_component = configuration_components[i];
    if (!configuration_component.hasMember("depends"))
    {
      continue;
    }

    if (configuration_component["depends"].getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      error = "The element #" + std::to_string(i) + " has 'depends' bad-formed, while an array was expected";
      return false;
    }

    std::string configuration_name = configuration_component["name"];
    for (size_t j = 0; j < configuration_component["depends"].size(); j++)
    {
      std::string dep_name = (std::string)configuration_component["depends"][j];
      
      if (name_to_index.find(dep_name) == name_to_index.end())
      {
        error = "There is a broken dependency.";
        configurations.clear();
        return false;
      }
      size_t              dep_index = name_to_index[ dep_name ];
      std::cout << "[ " << configuration_name << " ]  Dep Name: "<< dep_name << " at "<< dep_index << std::endl;
      std::string         what;
      ConfigurationStruct configuration_depend_from;
      if (!param::get_configuration_component_no_dependencies(configuration_components[ dep_index ], configuration_depend_from, what))
      {
        error = "Error in the configuration " + std::to_string(i) + ": " + what;
        configurations.clear();
        return false;
      }
      concat(configurations[ configuration_name ].components, configuration_depend_from.components);      
    }
  }
  #endif
