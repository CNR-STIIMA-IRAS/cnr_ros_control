#include "ros/ros.h"
#include <configuration_msgs/ListConfigurations.h>
#include <configuration_msgs/StartConfiguration.h>
#include <configuration_msgs/StopConfiguration.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Joy.h>
#include <subscription_notifier/subscription_notifier.h>
#include <cnr_logger/cnr_logger.h>


void gazebo_cb(const ros::WallTimerEvent& ev)
{
  ros::NodeHandle nh;
  std_srvs::Empty srv;
  ros::ServiceClient start_gazebo_srv  = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  start_gazebo_srv.call(srv);
  ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><Gazebo unpaused");
}

class Dispatch
{
protected:
  ros::ServiceClient m_start_config;
  configuration_msgs::StartConfiguration conf_srv;
  cnr_logger::TraceLogger& m_logger;
public:
  Dispatch(const std::string& configuration,const ros::ServiceClient& start_config, cnr_logger::TraceLogger& logger):
     m_start_config(start_config), m_logger(logger)
  {
    conf_srv.request.start_configuration=configuration;
    conf_srv.request.strictness=1;
    CNR_INFO(m_logger, "Constructor: default configuration " << conf_srv.request.start_configuration);

  }
  void dispatch(const ros::WallTimerEvent& e)
  {
    CNR_INFO(m_logger, "Timer Callback: starting configuration " << conf_srv.request.start_configuration);
    m_start_config.call(conf_srv);
    CNR_INFO(m_logger, "Timer Callback: starting configuration "
             << conf_srv.request.start_configuration << " " << ( conf_srv.response.ok ? "DONE" : "FAILED") );
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "configuration_dispatcher");
  ros::start();

  ros::NodeHandle nh("~");
  ros::ServiceClient list_config  = nh.serviceClient<configuration_msgs::ListConfigurations>("/configuration_manager/list_configurations");
  ros::ServiceClient start_config = nh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  ros::ServiceClient stop_config  = nh.serviceClient<configuration_msgs::StopConfiguration> ("/configuration_manager/stop_configuration");

  std::string n = ros::this_node::getName();
  n.erase(0,1);
  std::replace( n.begin(), n.end(), '/', '_');


  cnr_logger::TraceLogger logger( n, nh.getNamespace() );

  bool start_gazebo;
  if (!nh.getParam("start_gazebo",start_gazebo))
    start_gazebo=false;

  ros::WallTimer gazebo_timer;
  if (start_gazebo)
  {
    double gazebo_wait_time=4;
    if (!nh.getParam("gazebo_bringup_time",gazebo_wait_time))
      gazebo_wait_time=4;
    gazebo_timer=nh.createWallTimer(ros::WallDuration(gazebo_wait_time), gazebo_cb, true);

    CNR_INFO(logger, "Waiting " << gazebo_wait_time << "seconds before unpausing gazebo (timer started: " << gazebo_timer.hasStarted() <<") ");

  }
  else
  {
    CNR_INFO(logger, "Waiting for configuration_manager wake up...");
    list_config.waitForExistence();
    start_config.waitForExistence();
    stop_config.waitForExistence();

  }

  std::map<std::string,double> configurations;

  if (!nh.getParam("configuration_dispatches",configurations))
  {
    CNR_WARN(logger, "Nothing to dispatch");
    return 0;
  }


  std::vector<std::shared_ptr<Dispatch>> dispatches;
  std::vector<ros::WallTimer> dispatch_timers;
  for (const std::pair<std::string,double>&p : configurations)
  {
    dispatches.push_back(std::make_shared<Dispatch>(p.first,start_config, logger));
    dispatch_timers.push_back(nh.createWallTimer(ros::WallDuration(p.second),
                                             &Dispatch::dispatch,
                                             (dispatches.back()).get(),
                                             true)) ;
  }
  ros::spin();

  return 0;
}
