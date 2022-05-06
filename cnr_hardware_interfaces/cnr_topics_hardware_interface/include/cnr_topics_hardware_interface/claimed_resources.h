#ifndef CNR_TOPICS_HARDWARE_INTERFACE__CLAIMED_RESOURCES__H
#define CNR_TOPICS_HARDWARE_INTERFACE__CLAIMED_RESOURCES__H

#include <list>
#include <map>
#include <vector>
#include <string>
#include <mutex>

#include <ros/node_handle.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
 #include <cnr_hardware_interface/force_torque_state_interface.h>
 #include <cnr_hardware_interface/force_torque_command_interface.h>
#include <cnr_hardware_interface/analog_state_interface.h>
#include <cnr_hardware_interface/analog_command_interface.h>
#include <cnr_hardware_interface/pose_state_interface.h>
#include <cnr_hardware_interface/pose_command_interface.h>
#include <cnr_hardware_interface/twist_state_interface.h>
#include <cnr_hardware_interface/twist_command_interface.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace cnr_hardware_interface
{


enum RESOURCE_ID
{
  JOINT_RESOURCE, ANALOG_RESOURCE, WRENCH_RESOURCE, POSE_RESOURCE, TWIST_RESOURCE
};

inline const std::map<RESOURCE_ID, std::string>& RESOURCES()
{
  static std::map<RESOURCE_ID, std::string> ret = { {JOINT_RESOURCE, "joint_resource" }
    , {ANALOG_RESOURCE, "analog_resource" }
    , {WRENCH_RESOURCE, "wrench_resource" }
    , {POSE_RESOURCE, "pose_resource" }
    , {TWIST_RESOURCE, "twist_resource" }
  };
  return ret;
}
inline std::string AVAILABLE_RESOURCES()
{
  std::string ret;
  for (auto const & p : RESOURCES())
    ret += p.second + ", ";
  return ret;
}

struct Resource
{
  std::vector<std::string>  m_subscribed_topics;
  std::vector<std::string>  m_published_topics;
  double                    m_feedback_joint_state_timeout_s;
};

struct JointResource : Resource
{
  std::vector<std::string>  m_joint_names;
};

struct ForceTorqueResource : Resource
{
  std::string m_sensor_name;
  std::string m_frame_id;
};

struct AnalogResource : Resource
{
  std::vector<std::string>    m_channel_names;
};

struct PoseResource : Resource
{
  geometry_msgs::PoseStamped  m_pose;
  std::string                 m_frame_id;
};

struct TwistResource : Resource
{
  std::vector<std::string>  m_frames_id;

};

template < typename MSG >
struct ClaimedResource
{
  ClaimedResource() = delete;
  ClaimedResource(const cnr_hardware_interface::Resource&  res
                , const std::vector<std::string>&           resource_names
                , ros::NodeHandle&                          robothw_nh
                , std::map< std::string, bool> &            topics_received);

  virtual ~ClaimedResource();

  virtual void init();
  virtual void setParam(const std::string& ns);
  virtual void write(const ros::Time& time, const ros::Duration& period);
  virtual void callback(const typename MSG::ConstPtr& msg, const std::string& topic);
  virtual void shutdown();
  bool checkForConflict(const std::list< hardware_interface::ControllerInfo >& info) const;

  const std::string                                    m_namespace;
  std::map< std::string, bool>&                        m_topics_received;
  std::vector< std::shared_ptr<ros::Subscriber> >      m_sub;
  std::vector< std::shared_ptr<ros::Publisher>  >      m_pub;
  std::vector<typename MSG::Ptr>                       m_pub_msg;
  std::vector<std::string>                             m_resource_names;
  std::map< std::string, std::vector<std::string   > > m_resource_names_map;
  std::map< std::string, std::vector< unsigned int > > m_idxes_ax_map;

  std::mutex                                           m_mutex;
  size_t                                               m_msg_counter;
  bool                                                 m_shutted_down;

};

/**
 * 
 * 
 * 
 * 
 * 
 */
struct JointClaimedResource : ClaimedResource<sensor_msgs::JointState>
{
  JointClaimedResource() = delete;
  JointClaimedResource(const cnr_hardware_interface::JointResource& jr, ros::NodeHandle& robothw_nh, std::map< std::string, bool> & topics_received);

  void init();
  void shutdown();
  void setParam(const std::string& ns);
  void write(const ros::Time& time, const ros::Duration& period);
  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);
  void callback(const sensor_msgs::JointStateConstPtr& msg, const std::string& topic);

  hardware_interface::JointStateInterface         m_js_jh; //interface for reading joint state
  hardware_interface::PositionJointInterface      m_p_jh;  //interface for writing position target
  hardware_interface::VelocityJointInterface      m_v_jh;  //interface for writing velocity target
  hardware_interface::EffortJointInterface        m_e_jh;  //interface for writing effort target
  hardware_interface::PosVelEffJointInterface     m_pve_jh;
  hardware_interface::VelEffJointInterface        m_ve_jh;

  bool m_p_jh_active;
  bool m_v_jh_active;
  bool m_e_jh_active;


  std::vector<double> m_pos; // feedback position
  std::vector<double> m_vel; // feedback velocity
  std::vector<double> m_eff; // feedback effort

  std::vector<double> m_cmd_pos; //target position
  std::vector<double> m_cmd_vel; //target velocity
  std::vector<double> m_cmd_eff; //target effort

};

struct AnalogClaimedResource : ClaimedResource< std_msgs::Float64MultiArray >
{
  AnalogClaimedResource() = delete;
  AnalogClaimedResource(const cnr_hardware_interface::AnalogResource& ar, ros::NodeHandle& robothw_nh, std::map< std::string, bool> & topics_received);

  void init();
  void shutdown();
  void setParam(const std::string& ns);
  void write(const ros::Time& time, const ros::Duration& period);
  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);
  void callback(const std_msgs::Float64MultiArray::ConstPtr& msg, const std::string& topic);

  hardware_interface::AnalogStateInterface   m_a_sh; //interface for reading joint state
  hardware_interface::AnalogCommandInterface m_a_h;

  bool                                       m_a_h_active;
  bool                                       m_a_sh_active;

  std::vector<double>                        m_state;  // subscribed
  std::vector<double>                        m_output; // published
};

struct ForceTorqueClaimedResource : ClaimedResource< geometry_msgs::WrenchStamped >
{
  ForceTorqueClaimedResource() = delete;
  ForceTorqueClaimedResource(const cnr_hardware_interface::ForceTorqueResource& ar, ros::NodeHandle& robothw_nh, std::map< std::string, bool> & topics_received);

  void init();
  void shutdown();
  void setParam(const std::string& ns);
  void write(const ros::Time& time, const ros::Duration& period);
  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);
  void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg, const std::string& topic);

  hardware_interface::ForceTorqueSensorInterface m_w_sensor_h;  //interface for reading joint state complaint with ros_control
  hardware_interface::ForceTorqueStateInterface  m_w_sh; //interface for reading joint state
  hardware_interface::ForceTorqueInterface       m_w_h;

  bool                                        m_w_h_active;
  bool                                        m_w_sh_active;
  
  std::vector<double>                         m_state;  // subscribed
  std::vector<double>                         m_output; // published

  std::string                                 m_frame_id;
};

struct PoseClaimedResource : ClaimedResource< geometry_msgs::PoseStamped >
{
  PoseClaimedResource() = delete;
  PoseClaimedResource(const cnr_hardware_interface::PoseResource& ar, ros::NodeHandle& robothw_nh, std::map< std::string, bool> & topics_received);

  void init();
  void shutdown();
  void setParam(const std::string& ns);
  void write(const ros::Time& time, const ros::Duration& period);
  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);
  void callback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& topic);

  hardware_interface::PoseStateInterface      m_p_sh; //interface for reading joint state
  hardware_interface::PoseCommandInterface    m_p_h;

  bool                                        m_p_h_active;
  bool                                        m_p_sh_active;
  
  geometry_msgs::Pose                         m_state;  // subscribed
  geometry_msgs::Pose                         m_output; // published

};

struct TwistClaimedResource : ClaimedResource< geometry_msgs::TwistStamped >
{
  TwistClaimedResource() = delete;
  TwistClaimedResource(const cnr_hardware_interface::TwistResource& ar, ros::NodeHandle& robothw_nh, std::map< std::string, bool> & topics_received);

  void init();
  void shutdown();
  void setParam(const std::string& ns);
  void write(const ros::Time& time, const ros::Duration& period);
  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);
  void callback(const geometry_msgs::TwistStamped::ConstPtr& msg, const std::string& topic);

  std::vector<hardware_interface::TwistStateInterface >    m_t_sh; //interface for reading joint state
  std::vector<hardware_interface::TwistCommandInterface >  m_t_h;

  std::vector< bool >                                      m_t_sh_active;
  std::vector< bool >                                      m_t_h_active;

  std::vector<geometry_msgs::TwistStamped>    m_state;  // subscribed
  std::vector<geometry_msgs::TwistStamped>    m_output; // published

};


}  // namespace cnr_hardware_interface

#include "cnr_topics_hardware_interface/internal/claimed_resources_impl.h"

#endif  // CNR_TOPICS_HARDWARE_INTERFACE__CLAIMED_RESOURCES__H
