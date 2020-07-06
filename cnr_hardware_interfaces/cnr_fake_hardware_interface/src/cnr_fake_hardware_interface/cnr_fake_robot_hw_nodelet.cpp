
#include <pluginlib/class_list_macros.h>

#include <cnr_fake_hardware_interface/cnr_fake_robot_hw_nodelet.h>


PLUGINLIB_EXPORT_CLASS(cnr_hardware_nodelet_interface::FakeRobotHwNodelet, nodelet::Nodelet)

namespace cnr_hardware_nodelet_interface
{

bool FakeRobotHwNodelet::doOnInit()
{
  CNR_TRACE_START(*m_logger);
  try
  {
    m_hw.reset(new cnr_hardware_interface::FakeRobotHW());
  }
  catch( std::exception& e )
  {
    CNR_RETURN_FALSE(*m_logger, "OnInit of FakeRobotHw failed! what: " + std::string( e.what() ));
  }
  CNR_RETURN_TRUE(*m_logger);
}



}
