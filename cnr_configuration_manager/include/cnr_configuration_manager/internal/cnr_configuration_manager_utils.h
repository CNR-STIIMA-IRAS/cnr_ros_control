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
#ifndef CNR_CONFIGURATION_MANAGER_CNR_CONFIGURATION_MANAGER_UTILS_H
#define CNR_CONFIGURATION_MANAGER_CNR_CONFIGURATION_MANAGER_UTILS_H

#include <ros/ros.h>

#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletList.h>
#include <std_srvs/Trigger.h>
#include <boost/graph/graph_concepts.hpp>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <bondcpp/bond.h>
#include <mutex>

#include <configuration_msgs/ConfigurationComponent.h>
#include <configuration_msgs/ListConfigurations.h>
#include <configuration_msgs/StartConfiguration.h>
#include <configuration_msgs/StopConfiguration.h>

#include <cnr_configuration_manager/cnr_configuration_types.h>

namespace cnr_configuration_manager
{

struct RetrieveKey
{
  template <typename T>
  typename T::first_type operator()(T keyValuePair) const
  {
    return keyValuePair.first;
  }
};

template< typename T >
bool unique(std::vector< T >& vv)
{
  for (size_t i = 0; i < vv.size(); i++)
  {
    T check = vv[i];
    for (size_t j = i + 1; j < vv.size(); j++)
    {
      if (check == vv[j])
      {
        vv.erase(vv.begin() + j);
      }
    }
  }
  return true;
}


template< typename T >
bool equal(const std::vector< T >& lhs, const std::vector< T >& rhs)
{
  if (lhs.size() != rhs.size()) return false;
  for (auto const l : lhs)
  {
    if (std::find(rhs.begin(), rhs.end(), l) == rhs.end()) return false;
  }
  return true;
}

template< typename T >
void extract(const std::vector< T >& va
             , const std::vector< T >& vb
             , std::vector< T >* a_not_in_b = nullptr
             , std::vector< T >* b_not_in_a = nullptr
             , std::vector< T >* a_in_b     = nullptr)
{
  if (a_not_in_b) a_not_in_b->clear();
  if (b_not_in_a) b_not_in_a->clear();
  if (a_in_b) a_in_b    ->clear();

  if ((va.size() == 0) && (vb.size() == 0))
  {
    return;
  }
  else if (va.size() == 0)
  {
    if (a_not_in_b) a_not_in_b   ->clear();
    if (b_not_in_a) *b_not_in_a = vb;
    if (a_in_b) a_in_b      ->clear();
  }
  else if (vb.size() == 0)
  {
    if (a_not_in_b) *a_not_in_b = va;
    if (b_not_in_a) b_not_in_a  -> clear();
    if (a_in_b) a_in_b      ->clear();
  }
  else
  {
    for (auto const a : va)
    {
      if (std::find(vb.begin(), vb.end(), a) == vb.end())
      {
        if (a_not_in_b) a_not_in_b->push_back(a);
      }
      else
      {
        if (a_in_b) a_in_b->push_back(a);
      }
    }
    for (auto const b : vb)
    {
      if (std::find(va.begin(), va.end(), b) == va.end())
      {
        if (b_not_in_a) b_not_in_a->push_back(b);
      }
    }
  }
}

inline
void concat(ComponentMap& var, const ComponentMap&add)
{
  for (auto const & a : add)
  {
    const std::string &                key    = a.first;
    const std::vector<cnr::control::ControllerData>& values = a.second;
    if (var.find(key) == var.end())
    {
      var[ key ] = values;
    }
    else
    {
      for (auto const & value : values)
      {
        if (std::find(var[key].begin(), var[key].end(), value) == var[key].end())
        {
          var[key].push_back(value);
        }
      }
    }
  }
}

inline
void concat(ComponentMap& var, const std::pair< std::string, std::string >& add)
{
  const std::string & key   = add.first;
  cnr::control::ControllerData value;
  //const std::string&  value = add.second;
  value.id = add.second;
  value.check_state = true;
  if (var.find(key) == var.end())
  {
    var[ key ].push_back(value);
  }
  else
  {
    if (std::find(var[key].begin(), var[key].end(), value) == var[key].end())
    {
      var[key].push_back(value);
    }
  }
}

inline
std::vector<std::string> getHardwareInterfacesNames(const ConfigurationStruct& m)
{
  std::vector<std::string> ret(m.components.size());
  if(m.components.size()>0)
  {
    std::transform(m.components.begin(), m.components.end(), ret.begin(), RetrieveKey());
  }
  return ret;
}

inline
std::vector<std::string> getNames(const std::vector< controller_manager_msgs::ControllerState >& controllers)
{
  std::vector<std::string> ret;
  if(controllers.size()>0)
  {
    for (auto const & ctrl : controllers) 
      ret.push_back(ctrl.name);
  }
  return ret;
}

inline
bool cast(const configuration_msgs::ConfigurationComponent& in, cnr_configuration_manager::ConfigurationStruct& out)
{
  out.data.name  = in.name;
  out.data.state = in.state;
  out.data.type  = in.type;
  out.data.hidden = in.hidden;
  if (in.hardware_interfaces.size() != in.controllers.size())
  {
    return false;
  }

  for (size_t i = 0; i < in.hardware_interfaces.size(); i++)
  {
    cnr::control::ControllerData ctrl;
    ctrl.id = in.controllers[i];
    ctrl.check_state = in.check_runtime_status[i];
    out.components[ in.hardware_interfaces[i] ].push_back(ctrl);
  }

  for (auto & component : out.components)
  {
    ::cnr_configuration_manager::unique<cnr::control::ControllerData>(component.second);
  }
  return true;
}

inline bool cast(const ::cnr_configuration_manager::ConfigurationStruct& in, configuration_msgs::ConfigurationComponent& out)
{
  out.name   = in.data.name   ;
  out.state  = in.data.state  ;
  out.type   = in.data.type   ;
  out.hidden = in.data.hidden ;

  for (auto const & component : in.components)
  {
    for (auto const & controller : component.second)
    {
      out.hardware_interfaces.push_back(component.first);
      out.controllers.push_back(controller.id);
      out.check_runtime_status.push_back(controller.check_state);
    }
  }
  return true;
}




inline
bool extract_hardware_interfaces_names(const std::string& configuration_to_activate, const std::map< std::string, ConfigurationStruct >& configurations, std::vector<std::string>& hardware_interfaces, std::string error)
{
  if (configurations.find(configuration_to_activate) == configurations.end())
  {
    error = "The asked confgiuration '" + configuration_to_activate +  "' is not in the list of the configurations available ";
    return false;
  }

  hardware_interfaces.clear();
  hardware_interfaces.resize(configurations.at(configuration_to_activate).components.size());
  std::transform(configurations.at(configuration_to_activate).components.begin(), configurations.at(configuration_to_activate).components.end(), hardware_interfaces.begin(), RetrieveKey());
  return true;
}

/**
 * print utilities 
 */
template<typename T> inline
std::string to_string(const std::vector<T>& what,
                        const std::string& prefix = "<",
                          const std::string& separator = ",",
                            const std::string& suffix = ">" )
{
  std::stringstream ret; ret << prefix;
  for (size_t j = 0; j < what.size(); j++)
  {
    ret << what.at(j);
    ret << (j < what.size() - 1 ? separator : "");
  }
  ret << suffix;
  return ret.str();
}

template<> inline
std::string to_string(const std::vector<cnr::control::ControllerData>& what,
                        const std::string& prefix,
                          const std::string& separator,
                            const std::string& suffix )
{
  std::string ret = prefix;
  for (size_t j = 0; j < what.size(); j++)
  {
    std::string id = what.at(j).id;
    ret += id + (j < what.size() - 1 ? separator : "");
  }
  return ret + suffix;
}

inline 
std::string to_string( const ComponentData& in )
{
  return in.name + "[" + in.type + "](" + in.state +")";
}

inline 
std::string to_string( const ComponentMap& in, const std::string& prefix = "<", const std::string& separator = ",", const std::string& suffix = ">", bool nl = true )
{
  std::string ret = prefix + (nl ? "\n" : "");
  for(auto const & v : in ) ret += v.first + ":" + to_string(v.second, prefix, separator, suffix ) + (nl ? "\n" : "");
  return ret + suffix + (nl ? "\n" : "");
}

inline
std::string to_string( const ConfigurationStruct& what, const std::string& header = "Configuration:\n" )
{
  std::string ret = header;
  ret += "Data: " + to_string(what.data) + "\n";
  ret += "Components:\n" + to_string( what.components, "[",",","]",true)  +"\n";
  return ret;
}

inline 
std::string to_string(const std::map<std::string,ConfigurationStruct>& in )
{
  std::string ret;
  for(auto const & v : in ) ret += to_string( v.second, "Confiuration '" + v.first +"':\n");
  return ret;
}



}  // namespace cnr_configuration_manager

#endif  //   CNR_CONFIGURATION_MANAGER_CNR_CONFIGURATION_MANAGER_UTILS_H
