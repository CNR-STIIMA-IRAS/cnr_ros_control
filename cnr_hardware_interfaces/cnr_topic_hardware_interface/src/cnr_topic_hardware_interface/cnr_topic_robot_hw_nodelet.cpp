#include <cnr_topic_hardware_interface/cnr_topic_robot_hw_nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr_hardware_nodelet_interface::TopicRobotHwNodelet, nodelet::Nodelet)

namespace cnr_hardware_nodelet_interface
{
bool TopicRobotHwNodelet::doOnInit()
{
  CNR_TRACE_START(*m_logger);
  try
  {
    CNR_TRACE_START(*m_logger);
    std::vector<std::string> joint_names;
    if (!getPrivateNodeHandle().getParam("joint_names", joint_names))
    {
      CNR_FATAL(*m_logger, getPrivateNodeHandle().getNamespace()+"/joint_names' does not exist");
      CNR_FATAL(*m_logger, "ERROR DURING STARTING HARDWARE INTERFACE '" << getPrivateNodeHandle().getNamespace() << "'");
      CNR_RETURN_FALSE(*m_logger);
    }
    CNR_DEBUG(*m_logger, "Create the TopicRobotHW (joint names: " << joint_names.size() << ")");
    m_hw.reset(new cnr_hardware_interface::TopicRobotHW(/*joint_names*/));
    CNR_DEBUG(*m_logger, "TopicRobotHW Created");
  }
  catch( std::exception& e)
  {
    CNR_FATAL(*m_logger, getPrivateNodeHandle().getNamespace()+" exception in TopicRobotHw constructor. Abort: " << std::string( e.what() ) );
    CNR_RETURN_FALSE(*m_logger);
  }
  CNR_RETURN_TRUE(*m_logger);
}
}
