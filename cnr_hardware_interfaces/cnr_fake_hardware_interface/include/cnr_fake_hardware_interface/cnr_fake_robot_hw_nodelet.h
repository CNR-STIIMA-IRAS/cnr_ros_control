#ifndef __FAKE_HARDWARE_INTERFACE__ON_NODELET__
#define __FAKE_HARDWARE_INTERFACE__ON_NODELET__


#include <cnr_hardware_nodelet_interface/cnr_robot_hw_nodelet.h>
#include <cnr_fake_hardware_interface/cnr_fake_robot_hw.h>

namespace cnr_hardware_nodelet_interface
{

class FakeRobotHwNodelet : public cnr_hardware_nodelet_interface::RobotHwNodelet
{
public:
  virtual bool doOnInit();

protected:
};

}

#endif
