#include "ros/ros.h"
#include <configuration_msgs/ListConfigurations.h>
#include <configuration_msgs/StartConfiguration.h>
#include <configuration_msgs/StopConfiguration.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Joy.h>
#include <subscription_notifier/subscription_notifier.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "configuration_joy_interface");
  ros::NodeHandle nh;
  ros::Rate lp(100);
  ros::AsyncSpinner spin(4);
  spin.start();
  
  ros::ServiceClient list_config  = nh.serviceClient<configuration_msgs::ListConfigurations>("/configuration_manager/list_configurations");
  ros::ServiceClient start_config = nh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  ros::ServiceClient stop_config  = nh.serviceClient<configuration_msgs::StopConfiguration> ("/configuration_manager/stop_configuration");
  
  ros::ServiceClient srv_button0  = nh.serviceClient<std_srvs::SetBool>("/gripper/grasp");
  bool button0=false;
  
  ros::ServiceClient pause  = nh.serviceClient<std_srvs::SetBool>("/pause");
  ros::Publisher ovr_pub = nh.advertise<std_msgs::Int64>("speed_ovr",1);
  ros_helper::SubscriptionNotifier<sensor_msgs::Joy> joy_rec(nh,"joy",1);
  
  ROS_INFO("Waiting for configuration_manager wake up...");
  list_config.waitForExistence();
  start_config.waitForExistence();
  stop_config.waitForExistence();
  
  int ovr_value=10;
  std_msgs::Int64 ovr;
  for (int i=0;i<10;i++)
  {
    ovr_pub.publish(ovr);
    lp.sleep();
  }
  
  while ( (ros::ok()))
  {
    if (joy_rec.isANewDataAvailable())
    {
      sensor_msgs::Joy joy=joy_rec.getData();
      if (joy.buttons.at(0))
      {
        std_srvs::SetBool srv;
        button0=!button0;
        srv.request.data=button0;
        if (button0)
          ROS_INFO("grasp");
        else
          ROS_INFO("release");
        srv_button0.call(srv);
      }
      if (joy.buttons.at(4))
      {
        std_srvs::SetBool srv;
        
        srv.request.data=true;
        pause.call(srv);
      }
      if (joy.buttons.at(6))
      {
        std_srvs::SetBool srv;
        srv.request.data=false;
        pause.call(srv);
      }
      
      if (joy.buttons.at(5))
      {
        ovr.data+=5;
        ovr_value=std::min(100,std::max(0,ovr_value+1));
      }
      if (joy.buttons.at(7))
      {
        ovr.data+=5;
        ovr_value=std::min(100,std::max(0,ovr_value-1));
      }
      if (joy.buttons.at(0))
      {
        // switch configuration
      }
      if (joy.buttons.at(10))
      {
        configuration_msgs::StartConfiguration srv;
        srv.request.start_configuration="watch";
        start_config.call(srv);
      }
      if (joy.buttons.at(11))
      {
        configuration_msgs::StartConfiguration srv;
        srv.request.start_configuration="only_ur10";
        start_config.call(srv);
      }
      
    }
    ovr.data=ovr_value;
    ovr_pub.publish(ovr);
    lp.sleep();
  }
  return 0;
}
