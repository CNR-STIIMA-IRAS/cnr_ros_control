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
#ifndef CNR_HARDWARE_NODELET_INTERFACE_CNR_HARDWARE_NODELET_INTERFACE_H
#define CNR_HARDWARE_NODELET_INTERFACE_CNR_HARDWARE_NODELET_INTERFACE_H

#include <cnr_logger/cnr_logger.h>
#include <cnr_hardware_interface/internal/cnr_robot_hw_utils.h>
#include <cnr_controller_manager_interface/cnr_controller_manager_interface.h>
#include <cnr_hardware_driver_interface/cnr_hardware_driver_interface.h>
#include <cnr_configuration_manager/internal/cnr_configuration_manager_utils.h>

namespace cnr_configuration_manager
{


class ConfigurationLoader
{
private:
  ros::NodeHandle  root_nh_;

  std::map<std::string, cnr_hardware_driver_interface::RobotHwDriverInterfacePtr > drivers_;
  ConfigurationStruct  running_configuration_;

public:
  ConfigurationLoader(const ros::NodeHandle& root_nh);
  ~ConfigurationLoader()
  {
    drivers_.clear();
  }

  cnr_hardware_driver_interface::RobotHwDriverInterfacePtr getDriver(const std::string& hw)
  {
    return drivers_.find(hw) != drivers_.end() ? drivers_.at(hw) :  nullptr;
  }

  const cnr_hardware_driver_interface::RobotHwDriverInterfacePtr& getDriver(const std::string& hw) const
  {
    static cnr_hardware_driver_interface::RobotHwDriverInterfacePtr nullret; nullret.reset();
    return drivers_.find(hw) != drivers_.end() ? drivers_.at(hw) :  nullret;
  }

  const ConfigurationStruct& getRunningConfiguration() const
  {
    return running_configuration_;
  }

  ros::NodeHandle& getRootNh();

  /**
   */
  bool getHwParam(ros::NodeHandle &nh, const std::string& hw_name,
                    std::string&type, std::map<std::string,std::string>& remappings, std::string& error);
  
  /**
   */
  bool listHw(std::vector<std::string>& hw_names_from_nodelet, const ros::Duration& watchdog,
                std::string& error);
  
  /**
   */
  bool purgeHw(const ros::Duration& watchdog, std::string& error);
  
  /** @brief Loading of a single RobotHW (embedded in the RobotHwDriverInterfaces)
   */
  bool loadHw(const std::string& hw_to_load_name, const ros::Duration& watchdog, std::string& error);
  
  /** @brief Parallel Loading of a set of RobotHW (embedded in the RobotHwDriverInterfaces)
   */
  bool loadHw(const std::vector<std::string>& hw_to_load_names,
                const ros::Duration& watchdog, std::string& error);

  /** @brief Parallel Unloading of a set of RobotHW (embedded in the RobotHwDriverInterfaces)
   */
  bool unloadHw(const std::vector<std::string>& hw_to_unload_names, const ros::Duration& watchdog, 
                std::string& error);

  /** @brief Load of the RobotHW (if needed) and load and Start of the controllers for such RobotHW
   */ 
  bool loadAndStartControllers(const std::string& hw_name, const ConfigurationStruct& next_configuration,
                                const size_t& strictness, std::string& error);

  /** @brief Parallel Load of a set of RobotHW (if needed) and load and Start of the controllers for such RobotHW
   * 
   * NOTE: First the HW are loaded, if needed, and then the controllers are launched.
   */ 
  bool loadAndStartControllers(const std::vector<std::string>& hw_next_names,
                                const ConfigurationStruct& next_configuration, const size_t& strictness,
                                  std::string& error);

  /** @brief stop and unloads the controller, and unload the RobotHW
   */
  bool stopAndUnloadAllControllers(const std::vector<std::string>& hw_to_unload_names,
                                    const ros::Duration& watchdog, std::string& error);

  bool listControllers(const std::string& hw_name,
                        std::vector< controller_manager_msgs::ControllerState >& running,
                          std::vector< controller_manager_msgs::ControllerState >& stopped, 
                            std::string& error);
};

}  // namespace cnr_hardware_nodelet_interface

#endif  // CNR_HARDWARE_NODELET_INTERFACE_CNR_HARDWARE_NODELET_INTERFACE_H
