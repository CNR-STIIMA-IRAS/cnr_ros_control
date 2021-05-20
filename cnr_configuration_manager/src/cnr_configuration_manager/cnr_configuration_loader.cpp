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
  if (!nh.getParam(hw_name, hardware_interface))
  {
    error = "Param '" + nh.getNamespace()+ "/" + hw_name + "' does not exist";
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

  if ((hardware_interface.hasMember("remap_source_args"))
    &&(hardware_interface.hasMember("remap_target_args")) )
  {
    XmlRpc::XmlRpcValue remap_source_args = hardware_interface["remap_source_args"];
    XmlRpc::XmlRpcValue remap_target_args = hardware_interface["remap_target_args"];

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
      remappings[ros::names::resolve(from)] = ros::names::resolve(to);
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
  for (auto & driver : drivers_)
  {
    if(driver.second->getState() )
    if(!driver.second->getControllerManager()->stopUnloadAllControllers(watchdog))
    {
      error = "Error in stopping and unloading the controllers";
      return false;
    }
  }
  return true;
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
  for (auto old : hw_to_unload_names)
  {
    error += "hw: '" + old + "'\n";
    std::vector<controller_manager_msgs::ControllerState> running;
    std::vector<controller_manager_msgs::ControllerState> stopped;

    auto cm = drivers_[old]->getControllerManager();
    error += "get the ctrl list\n";
    if (!cm->listControllers(running, stopped, watchdog))
    {
      error += "error in getting the information of the status of the controllers." + cm->error();
      return false;
    }
    stopped.insert(stopped.end(), running.begin(), running.end());

    error += "ctrl doswitch (stop)\n";
    if (!cm->switchControllers(vc_empty, running, 1, watchdog))
    {
      error += "error in unloding the controllers: " + cm->error();
      return false;
    }

    error += "ctrl unload\n";
    if (!cm->unloadControllers(stopped, watchdog))
    {
      error += "error in unloading the controllers: " + cm->error();
      return false;
    }

    error += "hw nodelet unload (watchdog: " + std::to_string(watchdog.toSec()) + ")\n";
    try
    {
      drivers_[old].reset();
    }
    catch(const std::exception& e)
    {
      error += "Error in deleting the Robot Hardware Driver ...." + std::string(e.what());
      hw_set_state(getRootNh(), old, cnr_hardware_interface::SRV_ERROR);
      return false;
    }
    
    error += "set hw nodelet state to UNLOADED";
    hw_set_state(getRootNh(), old, cnr_hardware_interface::UNLOADED);
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
  // If the proper Controller Manager Interface does not exist, we created a neu CMI
  // The Controller Manager Interface, is like the standard ControlleManager.
  // Specifically: 
  // - A Controller Manager Proxy is created within the scope of the RobotHwNodelet. 
  //   When a new RobotHw must be created, a RobotHwNodelet is created (i.e., a nodelet is loaded by loadHw function)
  //   The RobotHwNodelet stores a Therefore, when a Controller Manager Proxy, that is a standard 
  //   Controller Manager, with the services exposed under a proper namespace
  // - The Controller Manager Interface is the 'client' side, that is configured to find the services offered by the 
  //   Controller Manager Proxy
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
  if (!drivers_[hw_name]->getControllerManager()->switchControllers(strictness, next_controllers, watchdog))
  {
    error = "Error in switching the controller: " 
            + drivers_[hw_name]->getControllerManager()->error();
    return false;
  }
  running_configuration_ = next_conf;
  //================================================

  return true;
}


bool ConfigurationLoader::loadAndStartControllers(const std::vector<std::string>& hw_next_names,
                                                  const ConfigurationStruct& next_conf,
                                                  const size_t& strictness, std::string& error)
{
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

  auto starter=[&](const std::string & hw_name, cnr_controller_manager_interface::ControllerManagerPtr ctrl,
                    const ConfigurationStruct& next, bool& ok, std::string& error)
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

      ok = ctrl->switchControllers(strictness, next_controllers, watchdog);
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
                                          drivers_.at(hw_name)->getControllerManager(),
                                            next_conf, std::ref(start_ok[hw_name]), std::ref(errors[hw_name]) );
  }

  for (auto& thread : starters)
  {
    auto cm = drivers_.at(thread.first)->getControllerManager();
    if (thread.second->joinable())
    {
      thread.second->join();
    }
    delete thread.second;
  }

  bool ok = true;
  std::for_each(start_ok.begin(), start_ok.end(), [&](std::pair<std::string, bool> b)
  {
    auto cm = drivers_.at(b.first)->getControllerManager();
    if(b.second)
    {
      //CNR_INFO(cm->getLogger(), "Successful starting the controllers of HW '" + b.first + "'");
    }
    else
    {
      error = "Error in starting the controllers of HW '" + b.first + "': " 
                  + cm->error() + " (" + errors[b.first] + ")";
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
  auto stopper=[&](const std::string & hw, cnr_controller_manager_interface::ControllerManagerPtr ctrl, 
                   bool& ok, std::string& error)
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
      ok = ctrl->stopUnloadAllControllers( null_watchdog ? ros::Duration(0.0) : ros::Duration(10.0) );
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
    stoppers[hw_name] = new std::thread(stopper, hw_name,  drivers_.at(hw_name)->getControllerManager(), 
                                        std::ref(unload_ok[hw_name]), std::ref(errors[hw_name]));
  }
  
  for (auto& thread : stoppers)
  {
    auto cm = drivers_.at(thread.first)->getControllerManager();
    if (thread.second->joinable())
    {
      thread.second->join();
    }
  }

  bool ok = true;
  std::for_each(unload_ok.begin(), unload_ok.end(), [&](std::pair<std::string, bool> b)
  {
    auto cm = drivers_.at(b.first)->getControllerManager();
    if(b.second)
    {
      //CNR_INFO(cm->getLogger(), "Successful stopping the controllers of HW '" + b.first + "'");
    }
    else
    {
      error = "Error in stopping the controllers of HW '" + b.first + "': " 
                + cm->error()+ "(" + errors[b.first]+")";
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
  
  if(!drivers_.at(hw_name)->getControllerManager()->listControllers(running, stopped, ros::Duration(1.0)))
  {
    error = drivers_.at(hw_name)->getControllerManager()->error();
    return false;
  }
  return true;
}


}  // namespace cnr_configuration_manager
