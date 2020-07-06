#ifndef __TOPIC_HARDWARE_INTERFACE__ON_NODELET__
#define __TOPIC_HARDWARE_INTERFACE__ON_NODELET__

#include <cnr_topics_hardware_interface/cnr_topics_robot_hw.h>
#include <cnr_hardware_nodelet_interface/cnr_robot_hw_nodelet.h>

namespace cnr_hardware_nodelet_interface
{
    
class TopicsRobotHwNodelet : public cnr_hardware_nodelet_interface::RobotHwNodelet
{
public:
  virtual bool doOnInit();

protected:
};
    
}

#endif
