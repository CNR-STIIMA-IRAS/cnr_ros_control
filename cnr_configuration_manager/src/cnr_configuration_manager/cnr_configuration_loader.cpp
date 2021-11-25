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
#include <iterator>
#include <vector>
#include <algorithm>
#include <thread>
#include <ros/ros.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <cnr_logger/cnr_logger.h>
#include <configuration_msgs/SendMessage.h>

#include <cnr_controller_manager_interface/cnr_controller_manager_interface.h>
#include <cnr_configuration_manager/cnr_configuration_loader.h>

namespace cnr_configuration_manager
{

ConfigurationLoader::ConfigurationLoader(const ros::NodeHandle& root_nh)
  : root_nh_(root_nh)
{
  drivers_.clear();
}

ros::NodeHandle& ConfigurationLoader::getRootNh()
{
  return root_nh_;
}

bool ConfigurationLoader::getHwParam(ros::NodeHandle &nh,
                                     const std::string& hw_name,
                                     std::string& type,
                                     std::map<std::string,std::string>& remappings,
                                     std::string& error)
{
  XmlRpc::XmlRpcValue hardware_interface;
  if (!rosparam_utilities::get(nh.getNamespace() +"/" + hw_name, hardware_interface, error))
  {
    return false;
  }
  if (!hardware_interface.hasMember("nodelet_type"))
  {
    //error = "The hardware_interface has not the field 'type'";
    type = "cnr/control/RobotHwNodelet";
  }
  else
  {
    type = (std::string)hardware_interface["nodelet_type"];
  }

  std::vector<std::string> postfix = {"","_0","_1","_2","_3","_4","_5","_6","_7","_8","_9","_10","_11","_12"};

  for(const auto & p : postfix )
  {
    if ((hardware_interface.hasMember("remap_source_args" + p))
      &&(hardware_interface.hasMember("remap_target_args" + p )) )
    {
      XmlRpc::XmlRpcValue remap_source_args = hardware_interface["remap_source_args" + p];
      XmlRpc::XmlRpcValue remap_target_args = hardware_interface["remap_target_args" + p];

      if (remap_source_args.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        error = "The remap_source_args is not a list of names" ;
        return false;
      }
      if (remap_target_args.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        error = "The remap_target_args is not a list of names" ;
        return false;
      }
      if (remap_source_args.size() != remap_target_args.size() )
      {
        error = "The remap_source_args and remap_target_args have a different dimension!" ;
        return false;
      }
      for (int i = 0; i < remap_source_args.size(); i++)
      {
        std::string from = (std::string)remap_source_args[i];
        std::string to = (std::string)remap_target_args[i];
        if(remappings.find(ros::names::resolve(from))==remappings.end())
        {
          remappings[ros::names::resolve(from)] = ros::names::resolve(to);
        }
        else
        {
          error += "The remap argument " + from + "[" + ros::names::resolve(from)
                    + "] has been already used. Check the configuration! Abort.";
          return false;
        }
      }
    }
  }
  return true;
}


bool ConfigurationLoader::listHw(std::vector<std::string>& hw_names_from_nodelet, const ros::Duration& watchdog, std::string& error)
{
  hw_names_from_nodelet.clear();
  for(auto const & d : drivers_) hw_names_from_nodelet.push_back(d.first); 
  return true;
}

bool ConfigurationLoader::purgeHw(const ros::Duration& watchdog, std::string& error)
{
  std::vector<std::string> hw_names;
  if(!listHw(hw_names, watchdog, error))
  {
    return false;
  }
  return unloadHw(hw_names, watchdog, error);
}

bool ConfigurationLoader::loadHw(const std::string& hw_to_load_name, 
                                  const ros::Duration& watchdog, std::string& error)
{
  return loadHw(std::vector<std::string>(1,hw_to_load_name),watchdog,error);
}

bool ConfigurationLoader::loadHw(const std::vector<std::string>& hw_to_load_names,
                                   const ros::Duration& watchdog,
                                    std::string& error)
{
  std::vector<std::string> hw_to_start_names;
  //=====================================================================================
  // INIT THE DRIVERS 
  ros::NodeHandle nh("/");
  for (auto const & hw_to_load_name : hw_to_load_names)
  {
    if(drivers_.find(hw_to_load_name) != drivers_.end() )
    {
      continue;
    }
    hw_to_start_names.push_back(hw_to_load_name);

    std::string type;
    std::map<std::string,std::string> remappings;
    if (!getHwParam(nh, hw_to_load_name, type, remappings, error))
    {
      error = "Loading The driver for RobotHW " + hw_to_load_name + " got an error: " + error;
      return false;
    }

    drivers_[hw_to_load_name].reset(new cnr_hardware_driver_interface::RobotHwDriverInterface() );
    if(!drivers_[hw_to_load_name]->init(hw_to_load_name, remappings))
    {
      error = "Failed when loading '" + hw_to_load_name + "'";
      return false;
    }
  }
  //=====================================================================================




  //=====================================================================================
  // START IN PARALLEL ALL THE HW
  std::map<std::string, std::thread* > starters;
  std::map< std::string, bool > start_ok;
  auto starter=[&](const std::string & hw_name, cnr_hardware_driver_interface::RobotHwDriverInterfacePtr driver,
                    bool& ok)
  {
    ok = driver->start(watchdog);
  };

  for (auto const & hw : hw_to_start_names)
  { 
    starters[hw] = new std::thread(starter, hw, drivers_[hw],std::ref(start_ok[hw]) );
  }

  for (auto& thread : starters)
  {
    if (thread.second->joinable())
    {
      thread.second->join();
    }
    delete thread.second;
  }

  bool ok = true;
  std::for_each(start_ok.begin(), start_ok.end(), [&](std::pair<std::string, bool> b)
  {
    if(!b.second)
    {
      error = "Error in starting the Driver of RobotHW '" + b.first +"'";
    }
    ok &= b.second;
  });
  //=====================================================================================

  return ok;
  //=====================================================================================
}

bool ConfigurationLoader::unloadHw(const std::vector<std::string>& hw_to_unload_names, 
                                    const ros::Duration& watchdog, std::string& error)
{
  static const std::vector<controller_manager_msgs::ControllerState> vc_empty;
  static const std::vector<std::string> vs_empty;
  
  error = "Loop over the hw: " + to_string(hw_to_unload_names) + "\n";
  for (auto hw : hw_to_unload_names)
  {
    try
    {
      auto it = drivers_.find(hw);
      if(it != drivers_.end())
      {
        if(!drivers_[hw]->getControllerManagerInterface()->stopUnloadAllControllers(watchdog))
        {
          error += "Failed in stopping and unloading the controllers";
          return false; 
        }
      
        error += "hw nodelet unload (watchdog: " + std::to_string(watchdog.toSec()) + ")\n";

        drivers_[hw].reset();
        drivers_.erase(it);
      }
    }
    catch(const std::exception& e)
    {
      error += "Error in deleting the Robot Hardware Driver ...." + std::string(e.what());
      cnr_hardware_interface::hw_set_state(hw, cnr_hardware_interface::SRV_ERROR);
      return false;
    }
    
    error += "set hw nodelet state to UNLOADED";
    cnr_hardware_interface::hw_set_state(hw, cnr_hardware_interface::UNLOADED);
  }
  return true;
}

bool ConfigurationLoader::loadAndStartControllers(const std::string& hw_name,
                                                  const ConfigurationStruct& next_conf,
                                                  const size_t& strictness,
                                                  std::string& error)
{
  //================================================
  // The following code is used to send a message that is written then in the log file
  // if (mail_senders_.find(hw_name) == mail_senders_.end())
  // {
  //   mail_senders_[hw_name] = root_nh_.serviceClient<configuration_msgs::SendMessage>("/"+hw_name+"/mail");
  // }
  // configuration_msgs::SendMessage srv;
  // srv.request.message.data = "========= "+next_conf.data.name+" ======== Load and Start Controllers ========";
  // if(!mail_senders_[hw_name].exists())
  // {
  //   CNR_ERROR(*logger_, "The service '" +mail_senders_[hw_name].getService() +"' does not exist... ?!?");
  // }
  // else
  // {
  //   mail_senders_[hw_name].call(srv);
  // }
  //================================================


  //================================================
  if (drivers_.find(hw_name) == drivers_.end())
  {
    loadHw(hw_name, ros::Duration(2.0), error);
  }
  //================================================


  //================================================
  std::vector<std::string> next_controllers = std::vector<std::string>();
  ros::Duration watchdog = ros::Duration(0.0);
  if(next_conf.components.find(hw_name) != next_conf.components.end())
  {
    next_controllers = extract_names(next_conf.components.at(hw_name));
    auto runtime_check = extract_runtime_checks(next_conf.components.at(hw_name));
    watchdog = std::find(runtime_check.begin(), runtime_check.end(), false) != runtime_check.end() 
             ? ros::Duration(0.0) : ros::Duration(2.0);
  }
  //================================================


  //================================================
  if (!drivers_[hw_name]->getControllerManagerInterface()->switchControllers(strictness, next_controllers, watchdog))
  {
    error = "Error in switching the controller: " 
            + drivers_[hw_name]->getControllerManagerInterface()->error();
    return false;
  }
  running_configuration_ = next_conf;
  //================================================

  return true;
}


bool ConfigurationLoader::loadAndStartControllers(const std::vector<std::string>& hw_next_names,
                                                  const ConfigurationStruct& next_conf,
                                                  const size_t& strictness, 
                                                  std::string& error)
{
  // PARALLEL START OF THE CONTROLLERS
  std::map<std::string, std::thread* > starters;
  std::map< std::string, bool > start_ok;
  for (auto const & hw_name : hw_next_names)
  {
    start_ok[hw_name] = false;
    if (drivers_.find(hw_name) == drivers_.end())
    {
      if(!loadHw(hw_name, ros::Duration(2.0), error))
      {
        return false;
      }
    }
  }

  // PARALLEL START OF THE CONTROLLERS
  auto starter=[&](const std::string & hw_name, const ConfigurationStruct& next, bool& ok, std::string& error)
  {
    try
    { 
      std::vector<std::string> next_controllers = std::vector<std::string>();
      ros::Duration watchdog = ros::Duration(0.0);
      
      if(next.components.find(hw_name) != next.components.end())
      {
        next_controllers = extract_names(next.components.at(hw_name));
        auto runtime_check = extract_runtime_checks(next.components.at(hw_name));
        watchdog = std::find(runtime_check.begin(), runtime_check.end(), false) != runtime_check.end() 
                              ? ros::Duration(0.0) : ros::Duration(2.0);
      }

      ok = drivers_[hw_name]->getControllerManagerInterface()->switchControllers(strictness,next_controllers,watchdog);
    }
    catch(std::exception& e)
    {
      error = "Exception in switch controllers. Error: " + std::string(e.what());
      ok = false;
    }
    catch(...)
    {
      error = "Unhandled Exception in switch controllers. ";
      ok = false;
    }
    return;
  };

  std::map<std::string, std::string> errors;
  for (auto const & hw_name : hw_next_names)
  { 
    errors[hw_name] = "";
    starters[hw_name] = new std::thread(starter, hw_name, 
                                            next_conf, std::ref(start_ok[hw_name]), std::ref(errors[hw_name]) );
  }

  for (auto& thread : starters)
  {
    if (thread.second->joinable())
    {
      thread.second->join();
    }
    delete thread.second;
  }

  bool ok = true;
  std::for_each(start_ok.begin(), start_ok.end(), [&](std::pair<std::string, bool> b)
  {
    if(!b.second)
    {
      error = "Error in starting the controllers of HW '" + b.first + "'";
    }
    ok &= b.second;
  });

  running_configuration_ = next_conf;
  return ok;
}


bool ConfigurationLoader::stopAndUnloadAllControllers(const std::vector<std::string>& hw_to_unload_names,
                                                        const ros::Duration& watchdog, std::string& error)
{
  std::map<std::string, std::thread* > stoppers;
  std::map< std::string, bool > unload_ok;

  for (auto const & hw_name : hw_to_unload_names)
  {
    unload_ok[hw_name] = false;
  }
  auto stopper=[&](const std::string & hw, bool& ok, std::string& error)
  {
    // if (mail_senders_.find(hw) == mail_senders_.end())
    // {
    //   mail_senders_[hw] = root_nh_.serviceClient<configuration_msgs::SendMessage>("/"+hw+"/mail");
    // }
    // configuration_msgs::SendMessage srv;
    // srv.request.message.data = "========= ======== UnLoad and Stop Controllers ========";
    // if(!mail_senders_[hw].exists())
    // {
    //   CNR_ERROR(*logger_, "The service '" +mail_senders_[hw].getService() +"' does not exist... ?!?");
    // }
    // else
    // {
    //   mail_senders_[hw].call(srv);
    // }
    try
    {
      bool null_watchdog = false;
      for(auto const & component : running_configuration_.components  )
      {
        auto runtime_check = extract_runtime_checks(component.second);
        null_watchdog  |= std::find(runtime_check.begin(), runtime_check.end(), false) != runtime_check.end();
      }
      ok = drivers_[hw]->getControllerManagerInterface()->stopUnloadAllControllers( null_watchdog ? ros::Duration(0.0) : ros::Duration(10.0) );
    }
   catch(std::exception& e)
    {
      error = "Exception in switch controllers. Error: " + std::string(e.what());
      ok = false;
    }
    catch(...)
    {
      error = "Unhandled Exception in switch controllers. ";
      ok = false;
    }
    return;
  };

  std::map<std::string,std::string> errors;
  for (auto const & hw_name : hw_to_unload_names)
  {
    errors[hw_name]="";
    stoppers[hw_name] = new std::thread(stopper, hw_name,
                                        std::ref(unload_ok[hw_name]), std::ref(errors[hw_name]));
  }
  
  for (auto& thread : stoppers)
  {
    if (thread.second->joinable())
    {
      thread.second->join();
    }
  }

  bool ok = true;
  std::for_each(unload_ok.begin(), unload_ok.end(), [&](std::pair<std::string, bool> b)
  {
    if(!b.second)
    {
      error = "Error in stopping the controllers of HW '" + b.first + "'";
    }
    ok &= b.second;
  });

  return ok;
}

bool ConfigurationLoader::listControllers(const std::string& hw_name,
                                          std::vector< controller_manager_msgs::ControllerState >& running,
                                          std::vector< controller_manager_msgs::ControllerState >& stopped, 
                                          std::string& error )
{
  if (drivers_.find(hw_name) == drivers_.end())
  {
    error = "robot hw '"+hw_name+"'not in the list of the robot hw loaded";
    return false;
  }
  
  if(!drivers_.at(hw_name)->getControllerManagerInterface()->listControllers(running, stopped, ros::Duration(1.0)))
  {
    error = drivers_.at(hw_name)->getControllerManagerInterface()->error();
    return false;
  }
  return true;
}


}  // namespace cnr_configuration_manager
