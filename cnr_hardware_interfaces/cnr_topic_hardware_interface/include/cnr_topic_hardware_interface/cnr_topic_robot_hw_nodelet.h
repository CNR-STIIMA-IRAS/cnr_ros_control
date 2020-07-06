#ifndef __TOPIC_HARDWARE_INTERFACE__ON_NODELET__
#define __TOPIC_HARDWARE_INTERFACE__ON_NODELET__

#include <controller_manager/controller_manager.h>
#include <nodelet/nodelet.h>
#include <thread>
#include <cnr_topic_hardware_interface/cnr_topic_robot_hw.h>
#include <cnr_hardware_nodelet_interface/cnr_robot_hw_nodelet.h>

namespace cnr_hardware_nodelet_interface
{
    
class TopicRobotHwNodelet : public cnr_hardware_nodelet_interface::RobotHwNodelet
{
public:
  virtual bool doOnInit();

protected:
};

}

#endif
