#ifndef CNR_ROS_CONTROL__CNR_CONFIGURATION_TYPES_H
#define CNR_ROS_CONTROL__CNR_CONFIGURATION_TYPES_H

#include <string>
#include <map>
#include <vector>
#include <cnr_controller_interface_params/cnr_controller_interface_params.h>

namespace cnr_configuration_manager
{


//!
struct ComponentData
{
  std::string name = "";
  std::string state = "";
  std::string type = "";
  bool        hidden = true;
};


//!
typedef std::map<std::string, std::vector<cnr::control::ControllerData> > ComponentMap;

//! For each
struct ConfigurationStruct
{
  ComponentData data;
  ComponentMap  components;
};



}  // namespace cnr_configuration_manager

#endif  // CNR_ROS_CONTROL__CNR_CONFIGURATION_TYPES_H