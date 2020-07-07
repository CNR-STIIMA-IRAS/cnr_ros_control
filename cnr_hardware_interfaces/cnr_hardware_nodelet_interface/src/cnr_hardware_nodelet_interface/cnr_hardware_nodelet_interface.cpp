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
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletList.h>
#include <cnr_logger/cnr_logger.h>

#include <cnr_controller_manager_interface/cnr_controller_manager_interface.h>
#include <cnr_hardware_nodelet_interface/cnr_hardware_nodelet_interface.h>

namespace cnr_hardware_nodelet_interface
{

NodeletManagerInterface::NodeletManagerInterface(std::shared_ptr<cnr_logger::TraceLogger> log, const std::string& root_ns, const std::string& nodelet_manager_ns)
  : root_nh_("/" + root_ns), logger_(log)
{
  ros::NodeHandle server_nh(nodelet_manager_ns);
  nodelet_loader_.reset(new nodelet::Loader(server_nh));

  load_       = root_nh_.serviceClient<nodelet::NodeletLoad> (nodelet_manager_ns + "/load_nodelet");
  unload_     = root_nh_.serviceClient<nodelet::NodeletUnload>(nodelet_manager_ns + "/unload_nodelet");
  list_       = root_nh_.serviceClient<nodelet::NodeletList> (nodelet_manager_ns + "/list");
}

ros::NodeHandle& NodeletManagerInterface::getNamespace()
{
  return root_nh_;
}
std::string      NodeletManagerInterface::loadServiceName()
{
  return load_  .getService();
}
std::string      NodeletManagerInterface::unloadServiceName()
{
  return unload_.getService();
}
std::string      NodeletManagerInterface::listServiceName()
{
  return list_  .getService();
}

bool NodeletManagerInterface::loadRequest(nodelet::NodeletLoad&   msg, std::string& error, const ros::Duration&  watchdog)
{
  return callRequest(load_, msg, error, watchdog);
}

bool NodeletManagerInterface::unloadRequest(nodelet::NodeletUnload& msg, std::string& error, const ros::Duration&  watchdog)
{
  return callRequest(unload_, msg, error, watchdog);
}
bool NodeletManagerInterface::listRequest(nodelet::NodeletList&   msg, std::string& error, const ros::Duration&  watchdog)
{
  return callRequest(list_, msg, error, watchdog);
}

bool NodeletManagerInterface::get_hw_param(ros::NodeHandle &nh, const std::string& hw_name, nodelet::NodeletLoadRequest& request)
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

bool NodeletManagerInterface::list_hw(std::vector<std::string>& hw_names_from_nodelet, const ros::Duration& watchdog)
{

  ::nodelet::NodeletList list_srv;
  if (!listRequest(list_srv, error_, watchdog))
  {
    error_ = "The service '" + listServiceName() + "' Failed. "  + error_;
    return false;
  }

  hw_names_from_nodelet = list_srv.response.nodelets;
  return true;
}

bool NodeletManagerInterface::purge_hw(const ros::Duration& watchdog)
{

  std::vector<std::string> hw_names_from_nodelet;
  if (!list_hw(hw_names_from_nodelet, watchdog))
  {
    return false;
  }


  for (const std::string& hw_name : hw_names_from_nodelet)
  {
    nodelet::NodeletUnload unload_srv;
    unload_srv.request.name = hw_name;
    std::string error_;
    if (!unloadRequest(unload_srv, error_, watchdog))
    {
      error_ = "The service '" + unloadServiceName() + "' Failed. " + error_;
      return false;
    }
    if (!unload_srv.response.success)
    {
      error_ = "The service '" + unloadServiceName() + "' failed in unload the nodelet '" + hw_name + "'. Abort.";
      return false;
    }
  }
  return true;
}



bool NodeletManagerInterface::load_hw(const std::string& hw_to_load_name, const ros::Duration& watchdog, bool double_check)
{

  nodelet::NodeletLoad load_srv;
  if (!get_hw_param(getNamespace(), hw_to_load_name, load_srv.request))
  {
    error_ = "Loading RobotHW [" + hw_to_load_name + "] error_: " + error_;
    return false;
  }
  if (!loadRequest(load_srv, error_, watchdog))
  {
    error_ = "The service '" + loadServiceName() + "' failed when loading '" + hw_to_load_name + "'. " + error_;
    return false;
  }
  if (!load_srv.response.success)
  {
    error_ = "The service '" + loadServiceName() + "' response with an error_ when loading '" + hw_to_load_name + "'";
    return false;
  }

  if (double_check)
  {
    std::vector<std::string> hw_names_from_nodelet;
    if (!list_hw(hw_names_from_nodelet, watchdog))
    {
      error_ = "error_ in getting the loaded hardware interfaces by the nodelet manager after loaded the new Hw: " + error_;
      return false;
    }

    if (std::find(hw_names_from_nodelet.begin(), hw_names_from_nodelet.end(), hw_to_load_name) == hw_names_from_nodelet.end())
    {
      error_  = "Something wrong happen: despite the loading service was correct, the actual loaded configuration by nodelet is different from the expected\n";
      error_ += "Asked Hw to load    : " + hw_to_load_name;
      error_ += "Loaded Hw by nodelet: " + to_string(hw_names_from_nodelet);
      return false;
    }
  }

  return true;
}



bool NodeletManagerInterface::load_hw(const std::vector<std::string>& hw_to_load_names, const ros::Duration& watchdog, bool double_check)
{

  CNR_TRACE_START(*logger_);
  size_t i = 0;
  for (auto const & next : hw_to_load_names)
  {
    nodelet::NodeletLoad load_srv;
    i++;
    if (!get_hw_param(getNamespace(), next, load_srv.request))
    {
      error_ = "Loading RobotHW [" + std::to_string(i) + "/" + std::to_string((int)hw_to_load_names.size()) + "] [" + next + "] error_: " + error_;
      CNR_RETURN_FALSE(*logger_);
    }
    if (!loadRequest(load_srv, error_, watchdog))
    {
      error_ = "The service '" + loadServiceName() + "' failed when loading '" + next + "'. " + error_;
      CNR_RETURN_FALSE(*logger_);
    }
    if (!load_srv.response.success)
    {
      error_ = "The service '" + loadServiceName() + "' response with an error_ when loading '" + next + "'";
      CNR_RETURN_FALSE(*logger_);
    }
  }

  if (double_check)
  {
    std::vector<std::string> hw_names_from_nodelet;
    if (!list_hw(hw_names_from_nodelet, watchdog))
    {
      error_ = "error_ in getting the loaded hardware interfaces by the nodelet manager after loaded the new Hw: " + error_;
      CNR_RETURN_FALSE(*logger_);
    }

    for (const std::string & hw : hw_to_load_names)
    {
      if (std::find(hw_names_from_nodelet.begin(), hw_names_from_nodelet.end(), hw) == hw_names_from_nodelet.end())
      {
        error_  = "Something wrong happen: despite the loading service was correct, the actual loaded configuration by nodelet is different from the expected\n";
        error_ += "Asked Hw to load    : " + to_string(hw_to_load_names);
        error_ += "Loaded Hw by nodelet: " + to_string(hw_names_from_nodelet);
        CNR_RETURN_FALSE(*logger_);
      }
    }
  }

  CNR_RETURN_TRUE(*logger_);
}


bool NodeletManagerInterface::unload_hw(const std::vector<std::string>& hw_to_unload_names, const ros::Duration& watchdog)
{

  CNR_TRACE_START(*logger_);
  CNR_DEBUG(*logger_, "Loop over the hw: " << to_string(hw_to_unload_names));
  for (auto old : hw_to_unload_names)
  {
    CNR_DEBUG(*logger_, "hw: '" << old << "'");
    std::vector<controller_manager_msgs::ControllerState >   running;
    std::vector<controller_manager_msgs::ControllerState>    stopped;

    CNR_DEBUG(*logger_, "get the ctrl list");
    if (!cmi_.at(old).list_ctrl(running, stopped, watchdog))
    {
      error_ = "error_ in getting the information of the status of the controllers." + cmi_.at(old).error();
      CNR_RETURN_FALSE(*logger_, cmi_.at(old).error());
    }
    stopped.insert(stopped.end(), running.begin(), running.end());

    CNR_DEBUG(*logger_, "ctrl doswitch (stop)");
    if (!cmi_.at(old).doswitch(1, nullptr, nullptr, &running, watchdog))
    {
      error_ = "error_ in unloding the controllers: " + cmi_.at(old).error();
      CNR_RETURN_FALSE(*logger_, error_);
    }

    CNR_DEBUG(*logger_, "ctrl unload");
    if (!cmi_.at(old).unload_ctrl(stopped, watchdog))
    {
      error_ = "error_ in unloding the controllers: " + cmi_.at(old).error();
      CNR_RETURN_FALSE(*logger_, error_);
    }

    CNR_DEBUG(*logger_, "hw nodelet unload (watchdog: " << watchdog.toSec() << ")");
    nodelet::NodeletUnload unload_srv;
    unload_srv.request.name = old;
    if (!unloadRequest(unload_srv, error_, watchdog))
    {
      error_ = "The service '" + unloadServiceName() + "' failed. " + error_;
      set_state(getNamespace(), old, cnr_hardware_interface::SRV_ERROR);
      CNR_RETURN_FALSE(*logger_, error_);
    }
    CNR_DEBUG(*logger_, "set hw nodelet state to UNLOADED");
    set_state(getNamespace(), old, cnr_hardware_interface::UNLOADED);
  }
  CNR_RETURN_TRUE(*logger_);
}




















}
