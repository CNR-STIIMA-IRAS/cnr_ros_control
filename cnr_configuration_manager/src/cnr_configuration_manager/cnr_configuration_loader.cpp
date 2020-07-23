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

#include <thread>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletList.h>
#include <cnr_logger/cnr_logger.h>

#include <cnr_controller_manager_interface/cnr_controller_manager_interface.h>
#include <cnr_configuration_manager/cnr_configuration_loader.h>

namespace cnr_configuration_manager
{

ConfigurationLoader::ConfigurationLoader(std::shared_ptr<cnr_logger::TraceLogger> log,
                                                 const std::string& root_ns)
  : root_nh_("/" + root_ns), logger_(log)
{
  nodelet_loader_.reset(new nodelet::Loader(false));
}

ros::NodeHandle& ConfigurationLoader::getNamespace()
{
  return root_nh_;
}

bool ConfigurationLoader::getHwParam(ros::NodeHandle &nh,
                                     const std::string& hw_name,
                                     nodelet::NodeletLoadRequest& request)
{

  XmlRpc::XmlRpcValue hardware_interface;
  if (!nh.getParam(hw_name, hardware_interface))
  {
    error_ = "Param '" + hw_name + "' does not exist";
    return false;
  }
  if (!hardware_interface.hasMember("nodelet_type"))
  {
    //error_ = "The hardware_interface has not the field 'type'";
    request.type = "cnr/control/RobotHwNodelet";
  }
  else
  {
    request.type    = (std::string)hardware_interface["nodelet_type"];
  }

  request.name    = hw_name;

  if (hardware_interface.hasMember("remap_source_args"))
  {
    XmlRpc::XmlRpcValue remap_source_args = hardware_interface["remap_source_args"];

    if (remap_source_args.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      error_ = "The remap_source_args is not a list of names" ;
      return false;
    }
    request.remap_source_args.resize(remap_source_args.size());
    for (int i = 0; i < remap_source_args.size(); i++)
      request.remap_source_args.at(i) = (std::string)remap_source_args[i];
  }

  if (hardware_interface.hasMember("remap_target_args"))
  {
    XmlRpc::XmlRpcValue remap_target_args = hardware_interface["remap_target_args"];

    if (remap_target_args.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      error_ = "The remap_target_args is not a list of names" ;
      return false;
    }
    if ((int)request.remap_source_args.size() != remap_target_args.size())
    {
      error_ = "remap_target_args is different w.r.t. remap_target_args";
      return false;
    }
    request.remap_target_args.resize(remap_target_args.size());
    for (int i = 0; i < remap_target_args.size(); i++)
      request.remap_target_args.at(i) = (std::string)remap_target_args[i];
  }
  return true;
}


bool ConfigurationLoader::getHwParam(ros::NodeHandle &nh,
                                           const std::string& hw_name,
                                           std::string& type,
                                           nodelet::M_string& remappings)
{

  XmlRpc::XmlRpcValue hardware_interface;
  if (!nh.getParam(hw_name, hardware_interface))
  {
    error_ = "Param '" + hw_name + "' does not exist";
    return false;
  }
  if (!hardware_interface.hasMember("nodelet_type"))
  {
    //error_ = "The hardware_interface has not the field 'type'";
    type = "cnr/control/RobotHwNodelet";
  }
  else
  {
    type    = (std::string)hardware_interface["nodelet_type"];
  }

  if ((hardware_interface.hasMember("remap_source_args"))
    &&(hardware_interface.hasMember("remap_target_args")) )
  {
    XmlRpc::XmlRpcValue remap_source_args = hardware_interface["remap_source_args"];
    XmlRpc::XmlRpcValue remap_target_args = hardware_interface["remap_target_args"];

    if (remap_source_args.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      error_ = "The remap_source_args is not a list of names" ;
      return false;
    }
    if (remap_target_args.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      error_ = "The remap_target_args is not a list of names" ;
      return false;
    }
    if (remap_source_args.size() != remap_target_args.size() )
    {
      error_ = "The remap_source_args and remap_target_args have a different dimension!" ;
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


bool ConfigurationLoader::listHw(std::vector<std::string>& hw_names_from_nodelet, const ros::Duration& watchdog)
{

  hw_names_from_nodelet = nodelet_loader_->listLoadedNodelets();
  return true;
}

bool ConfigurationLoader::purgeHw(const ros::Duration& watchdog)
{

  std::vector<std::string> hw_names_from_nodelet = nodelet_loader_->listLoadedNodelets();

  for (const std::string& hw_name : hw_names_from_nodelet)
  {
    if (!nodelet_loader_->unload(hw_name))
    {
      error_ = "The NodeletLoader failed in unload the nodelet '" + hw_name + "'. Abort.";
      return false;
    }
  }

  nodelet_loader_->clear();
  return true;
}



bool ConfigurationLoader::loadHw(const std::string& hw_to_load_name, const ros::Duration& watchdog, bool double_check)
{
  return loadHw(std::vector<std::string>(1,hw_to_load_name),watchdog, double_check);
}



bool ConfigurationLoader::loadHw(const std::vector<std::string>& hw_to_load_names,
                                 const ros::Duration& watchdog,
                                 bool double_check)
{

  CNR_TRACE_START(*logger_);
  size_t i = 0;
  for (auto const & hw_to_load_name : hw_to_load_names)
  {
    std::string type;
    nodelet::M_string remappings;
    nodelet::V_string my_argv;
    i++;
    if (!getHwParam(getNamespace(), hw_to_load_name, type, remappings))
    {
      error_ = ("Loading RobotHW [" + std::to_string(i) + "/" + std::to_string((int)hw_to_load_names.size()) + "]") +
               ("[" + hw_to_load_name + "] error_: " + error_);
      CNR_RETURN_FALSE(*logger_);
    }
    CNR_DEBUG(*logger_, "Namespace: " << getNamespace().getNamespace() << " Hw to load: " << hw_to_load_name
              << " type (from param): " << type );

    if (!nodelet_loader_->load(hw_to_load_name, type, remappings, my_argv))
    {
      error_ = "The NodeleLoader failed when loading '" + hw_to_load_name + "'";
      return false;
    }
  }

  if (double_check)
  {
    std::vector<std::string> hw_names_from_nodelet;
    if (!listHw(hw_names_from_nodelet, watchdog))
    {
      error_ = "error_ in getting the loaded hardware interfaces by the nodelet loader: " + error_;
      CNR_RETURN_FALSE(*logger_);
    }

    for (const std::string & hw : hw_to_load_names)
    {
      if (std::find(hw_names_from_nodelet.begin(), hw_names_from_nodelet.end(), hw) == hw_names_from_nodelet.end())
      {
        error_  = "Something wrong happen: despite the loading service was correct,";
        error_ += "the actual loaded configuration by nodelet is different from the expected\n";
        error_ += "Asked Hw to load    : " + to_string(hw_to_load_names);
        error_ += "Loaded Hw by nodelet: " + to_string(hw_names_from_nodelet);
        CNR_RETURN_FALSE(*logger_);
      }
    }
  }

  CNR_RETURN_TRUE(*logger_);
}


bool ConfigurationLoader::unloadHw(const std::vector<std::string>& hw_to_unload_names, const ros::Duration& watchdog)
{

  CNR_TRACE_START(*logger_);
  CNR_DEBUG(*logger_, "Loop over the hw: " << to_string(hw_to_unload_names));
  for (auto old : hw_to_unload_names)
  {
    CNR_DEBUG(*logger_, "hw: '" << old << "'");
    std::vector<controller_manager_msgs::ControllerState> running;
    std::vector<controller_manager_msgs::ControllerState> stopped;

    CNR_DEBUG(*logger_, "get the ctrl list");
    if (!cmi_.at(old).listControllers(running, stopped, watchdog))
    {
      error_ = "error_ in getting the information of the status of the controllers." + cmi_.at(old).error();
      CNR_RETURN_FALSE(*logger_, cmi_.at(old).error());
    }
    stopped.insert(stopped.end(), running.begin(), running.end());

    CNR_DEBUG(*logger_, "ctrl doswitch (stop)");
    if (!cmi_.at(old).switchControllers(1, nullptr, nullptr, &running, watchdog))
    {
      error_ = "error_ in unloding the controllers: " + cmi_.at(old).error();
      CNR_RETURN_FALSE(*logger_, error_);
    }

    CNR_DEBUG(*logger_, "ctrl unload");
    if (!cmi_.at(old).unloadControllers(stopped, watchdog))
    {
      error_ = "error_ in unloding the controllers: " + cmi_.at(old).error();
      CNR_RETURN_FALSE(*logger_, error_);
    }

    CNR_DEBUG(*logger_, "hw nodelet unload (watchdog: " << watchdog.toSec() << ")");
    if (!nodelet_loader_->unload( old ) )
    {
      error_ = "The nodelet loader failed. in unloadinf " + old;
      set_state(getNamespace(), old, cnr_hardware_interface::SRV_ERROR);
      CNR_RETURN_FALSE(*logger_, error_);
    }
    CNR_DEBUG(*logger_, "set hw nodelet state to UNLOADED");
    set_state(getNamespace(), old, cnr_hardware_interface::UNLOADED);
  }
  CNR_RETURN_TRUE(*logger_);
}

bool ConfigurationLoader::loadAndStartControllers(const std::string&              hw_name,
                                                  const ConfigurationStruct*      next_configuration,
                                                  const size_t                    strictness)
{
  // create proper configured servers
  if (cmi_.find(hw_name) == cmi_.end())
  {
    cmi_.emplace(hw_name, cnr_controller_manager_interface::ControllerManagerInterface(logger_, hw_name));
  }

  const std::vector<std::string>* next_controllers = ( next_configuration ?
                                                    & (next_configuration->components.at(hw_name))
                                                    : nullptr );
  CNR_DEBUG(*logger_, "List of controllers to be uploaded for the RobotHw: " << hw_name
                      << " next controllers: " << (next_controllers ?  to_string(*next_controllers) : "none"));
  if (!cmi_.at(hw_name).switchControllers(strictness, next_controllers, ros::Duration(10.0)))
  {
    CNR_RETURN_FALSE(*logger_, "Error in switching the controller");
  }
}


bool ConfigurationLoader::loadAndStartControllers(const std::vector<std::string>& hw_next_names,
                                                  const ConfigurationStruct*      next_configuration,
                                                  const size_t                    strictness)
{
  std::map<std::string, std::thread* > starters;
  std::map< std::string, bool > start_ok;
  for (auto const & hw_name : hw_next_names)
  {
    start_ok[hw_name] = false;
    if (cmi_.find(hw_name) == cmi_.end())
    {
      cmi_.emplace(hw_name, cnr_controller_manager_interface::ControllerManagerInterface(logger_, hw_name));
    }
  }

  auto starter=[&](const std::string & hw, cnr_controller_manager_interface::ControllerManagerInterface& ctrl,
                   const std::vector<std::string>* next_controllers, bool& ok)
  {
    CNR_INFO(*(ctrl.getLogger()), "Try starting the controllers of HW '" + hw + "'");
    ok = ctrl.switchControllers(strictness, next_controllers, ros::Duration(0.1));
    if (ok)
    {
      CNR_INFO(*(ctrl.getLogger()), "Successful starting the controllers of HW '" + hw + "'");
    }
    else
    {
      CNR_ERROR(*(ctrl.getLogger()), "Error in starting the controllers of HW '" + hw + "': " + ctrl.error());
    }
  };


  for (auto const & hw_name : hw_next_names)
  {
    const std::vector<std::string>* next_controllers = ( next_configuration ?
                                                      & (next_configuration->components.at(hw_name))
                                                      : nullptr );

    starters[hw_name] = new std::thread(starter, hw_name,  std::ref(cmi_.at(hw_name)),
                                        next_controllers, std::ref(start_ok[hw_name]));
  }

  CNR_DEBUG(*logger_,  "Waiting for threads");
  for (auto& thread : starters)
  {
    CNR_DEBUG(*(cmi_.at(thread.first).getLogger()), thread.first << ": Waiting for thread join execution ...");
    if (thread.second->joinable())
    {
      thread.second->join();
    }
    CNR_DEBUG(*(cmi_.at(thread.first).getLogger()), thread.first << ": Thread Finished ");
  }
  CNR_DEBUG(*logger_,  "Threads Joined");

  bool ok = true;
  std::for_each(start_ok.begin(), start_ok.end(), [&](std::pair<std::string, bool> b)
  {
    ok &= b.second;
  });

  CNR_RETURN_BOOL(*logger_, ok);
}


bool ConfigurationLoader::stopAndUnloadAllControllers( const std::vector<std::string>& hw_to_unload_names,
                                                    const ros::Duration&            watchdog)
{
  CNR_TRACE_START(*logger_);

  std::map<std::string, std::thread* > stoppers;
  std::map< std::string, bool > unload_ok;

  for (auto const & hw_name : hw_to_unload_names)
  {
    unload_ok[hw_name] = false;
  }
  auto stopper=[&](const std::string & hw, cnr_controller_manager_interface::ControllerManagerInterface& ctrl, bool& ok)
  {
    CNR_INFO(*(ctrl.getLogger()), "Try stopping the controllers of HW '" + hw + "'");
    ok = ctrl.stopUnloadAllControllers(watchdog);
    if (ok)
    {
      CNR_INFO(*(ctrl.getLogger()), "Successful stopping the controllers of HW '" + hw + "'");
    }
    else
    {
      CNR_ERROR(*(ctrl.getLogger()), "Error in stopping the controllers of HW '" + hw + "': " + ctrl.error());
    }
  };

  for (auto const & hw_name : hw_to_unload_names)
  {
    stoppers[hw_name] = new std::thread(stopper, hw_name,  std::ref(cmi_.at(hw_name)), std::ref(unload_ok[hw_name]));
  }
  CNR_DEBUG(*logger_,  "Waiting for threads");
  for (auto& thread : stoppers)
  {
    CNR_DEBUG(*(cmi_.at(thread.first).getLogger()), thread.first << ": Waiting for thread join execution ...");
    if (thread.second->joinable())
    {
      thread.second->join();
    }
    CNR_DEBUG(*(cmi_.at(thread.first).getLogger()), thread.first << ": Thread Finished ");
  }
  CNR_DEBUG(*logger_,  "Threads Joined");

  bool ok = true;
  std::for_each(unload_ok.begin(), unload_ok.end(), [&](std::pair<std::string, bool> b)
  {
    ok &= b.second;
  });

  CNR_RETURN_BOOL(*logger_, ok);
}

bool ConfigurationLoader::listControllers(const std::string& hw_name,
                                          std::vector< controller_manager_msgs::ControllerState >& running,
                                          std::vector< controller_manager_msgs::ControllerState >& stopped )
{
  if (cmi_.find(hw_name) == cmi_.end())
    return false;

  return cmi_.at(hw_name).listControllers(running, stopped, ros::Duration(1.0));
}


}  // namespace cnr_configuration_manager
