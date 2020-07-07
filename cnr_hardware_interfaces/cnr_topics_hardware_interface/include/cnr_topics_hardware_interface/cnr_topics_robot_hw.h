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
#ifndef CNR_HARDWARE_INTERFACE_CNR_TOPICS_ROBOT_HW_H
#define CNR_HARDWARE_INTERFACE_CNR_TOPICS_ROBOT_HW_H

#include <hardware_interface/joint_command_interface.h>

#include <name_sorting/name_sorting.h>

#include <cnr_hardware_interface/cnr_robot_hw.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>

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
#include <mutex>


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
                  , std::map< std::string, bool> &            topics_received)
    : m_robothw_nh(robothw_nh)
    , m_console(robothw_nh.getNamespace())
    , m_topics_received(topics_received)
    , m_resource_names(resource_names)
    , m_msg_counter(0)
    , m_shutted_down(false)
  {
    size_t l = __LINE__;
    try
    {
      for (size_t i = 0; i < res.m_subscribed_topics.size(); i++)
      {
        m_topics_received [ res.m_subscribed_topics.at(i) ] = false;
      }
      for (const std::string & subscribed_topic : res.m_subscribed_topics)
      {
        std::shared_ptr< ros::Subscriber > sub(new ros::Subscriber());
        *sub = robothw_nh.subscribe< MSG >(subscribed_topic, 1, boost::bind(&cnr_hardware_interface::ClaimedResource< MSG >::callback, this, _1, subscribed_topic));
        m_sub.push_back(sub);
      }

      m_pub.clear();
      if (res.m_published_topics.size() > 0)
      {
        m_pub.resize(res.m_published_topics.size());
        m_pub_msg.resize(res.m_published_topics.size());
        for (size_t i = 0; i < m_pub.size(); i++)
        {
          m_pub.at(i).reset(new ros::Publisher());
          *m_pub.at(i) = robothw_nh.advertise< MSG >(res.m_published_topics.at(i), 1);
          typename MSG::Ptr msg(new MSG());
          m_pub_msg.at(i) = msg;
        }
      }

      for (size_t i = 0; i < res.m_subscribed_topics.size(); i++)
      {
        const std::string & t = res.m_subscribed_topics.at(i);
        m_idxes_ax_map        [ t ].push_back(i);
        m_resource_names_map  [ t ].push_back(m_resource_names.at(i));
      }
    }
    catch (std::exception& e)
    {
      ROS_FATAL("ClaimedResource error %s (at line:%zu)", e.what(), l);
      std::cout.flush();
      fflush(stdout);
      fflush(stderr);
      throw std::runtime_error(("ClaimedResource error"  + std::string(e.what())) .c_str());
    }
    catch (...)
    {
      ROS_FATAL("ClaimedResource error. Unhandled expcetion at line:%zu", l);
      std::cout.flush();
      fflush(stdout);
      fflush(stderr);
      throw std::runtime_error("ClaimedResource error");
    }
  }

  ~ClaimedResource()
  {
    m_mutex.lock();
    m_mutex.unlock();
  }

  virtual void init()
  {

  }
  virtual void setParam(const std::string& ns)
  {

  }
  virtual void write(const ros::Time& time, const ros::Duration& period)
  {
    if (m_pub.size() > 0)
    {
      for (size_t i = 0; i < m_pub.size(); i++)
      {
        m_mutex.lock();
        m_pub.at(i)->publish(m_pub_msg.at(i));
        m_mutex.unlock();
        typename MSG::Ptr msg(new MSG());
        m_pub_msg.at(i).swap(msg);
      }
    }
  }
  virtual void callback(const typename MSG::ConstPtr& msg, const std::string& topic)
  {
    m_msg_counter++;
  }
  virtual void shutdown()
  {
    if (!m_shutted_down)
    {
      if (m_pub.size() > 0)
      {
        for (size_t i = 0; i < m_pub.size(); i++)
        {
          m_pub.at(i).reset();
        }
        m_pub.clear();
      }
      m_shutted_down = true;
    }
  }
  bool checkForConflict(const std::list< hardware_interface::ControllerInfo >& info)
  {
    std::vector<bool> global_joint_used(m_resource_names.size());
    std::fill(global_joint_used.begin(), global_joint_used.end(), false);

    for (hardware_interface::ControllerInfo controller : info)
    {
      std::vector<bool> single_controller_joint_used(m_resource_names.size());
      std::fill(single_controller_joint_used.begin(), single_controller_joint_used.end(), false);

      for (hardware_interface::InterfaceResources res : controller.claimed_resources)
      {
        for (std::string name : res.resources)
        {
          for (unsigned int iJ = 0; iJ < m_resource_names.size(); iJ++)
          {
            if (!name.compare(m_resource_names.at(iJ)))
            {
              if (global_joint_used.at(iJ)) // if already used by another
              {
                return true;
              }
              else
                single_controller_joint_used.at(iJ) = true;
            }
          }
        }
      }
      for (unsigned int iJ = 0; iJ < m_resource_names.size(); iJ++)
        global_joint_used.at(iJ) = global_joint_used.at(iJ) || single_controller_joint_used.at(iJ);
    }
    return false;
  }

  ros::NodeHandle&                                     m_robothw_nh;
  const std::string                                    m_console;
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

struct JointClaimedResource : ClaimedResource< sensor_msgs::JointState >
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

  unsigned int m_nAx;
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

  bool                                       m_a_sh_active;
  bool                                       m_a_h_active;

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

  hardware_interface::ForceTorqueStateInterface  m_w_sh; //interface for reading joint state
  hardware_interface::ForceTorqueInterface       m_w_h;

  bool                                        m_w_sh_active;
  bool                                        m_w_h_active;

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

  bool                                        m_p_sh_active;
  bool                                        m_p_h_active;

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

/**
 *
 *
 *
 * @class TopicsRobotHW
 *
 *
 *
 */

class TopicsRobotHW: public cnr_hardware_interface::RobotHW
{
public:
  TopicsRobotHW();
  virtual ~TopicsRobotHW()
  {
    if (!m_shutted_down)
      shutdown();
  }
  virtual bool doShutdown();
  virtual bool doRead(const ros::Time& time, const ros::Duration& period);
  virtual bool doWrite(const ros::Time& time, const ros::Duration& period);

  virtual bool doInit() ;
  virtual bool doCheckForConflict(const std::list<hardware_interface::ControllerInfo>& info) ;
  virtual bool doPrepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);


protected:

  std::map< RESOURCE_ID, std::shared_ptr< cnr_hardware_interface::Resource > > m_resources;


  std::shared_ptr< JointClaimedResource > m_joint_resource;
  bool initJointClaimedResource();

  std::shared_ptr< AnalogClaimedResource > m_analog_resource;
  bool initAnalogClaimedResource();

  std::shared_ptr< ForceTorqueClaimedResource > m_force_torque_sensor_resource;
  bool initForceTorqueClaimedResource();

  std::shared_ptr< PoseClaimedResource > m_pose_resource;
  bool initPoseClaimedResource();

  std::shared_ptr< TwistClaimedResource > m_twist_resource;
  bool initTwistClaimedResource();


  enum status { created, initialized, run, error };

  unsigned int                  m_missing_messages;
  unsigned int                  m_max_missing_messages;
  std::map< std::string, bool>  m_topics_subscribed;

  bool allSubscriberConnected() const
  {
    bool all_topics_received = true;
    for (const std::pair<std::string, bool>& topic_received : m_topics_subscribed) all_topics_received &= topic_received.second;
    return all_topics_received;
  }

  bool topicsReceived() const
  {
    bool all_topics_received = true;
    for (const std::pair<std::string, bool>& topic_received : m_topics_subscribed) all_topics_received &= topic_received.second;
    return all_topics_received;
  }
  void resetTopicsReceived()
  {
    for (auto & topic_received : m_topics_subscribed) topic_received.second = false;
  }

  friend void setParam(TopicsRobotHW* hw, const std::string& ns);

};

void setParam(TopicsRobotHW* hw, const std::string& ns);
std::vector<std::string> getResourceNames(const std::map< cnr_hardware_interface::RESOURCE_ID, std::shared_ptr< cnr_hardware_interface::Resource > >& resources) ;





















/**
 *
 *
 *
 *
 *
 *
 *
 *
 */
inline
JointClaimedResource::JointClaimedResource(const cnr_hardware_interface::JointResource& jr,
                                           ros::NodeHandle& robothw_nh, 
                                           std::map<std::string, bool> & topics_received)
: cnr_hardware_interface::ClaimedResource<sensor_msgs::JointState> (jr, jr.m_joint_names, robothw_nh, topics_received), 
m_nAx(0),
m_p_jh_active(false),
m_v_jh_active(false),
m_e_jh_active(false),
{
  m_nAx  = m_resource_names.size();
  m_pos.resize(m_nAx, 0);
  m_vel.resize(m_nAx, 0);
  m_eff.resize(m_nAx, 0);

  m_cmd_pos.resize(m_nAx, 0);
  m_cmd_vel.resize(m_nAx, 0);
  m_cmd_eff.resize(m_nAx, 0);

  m_cmd_pos = m_pos;
  m_cmd_vel = m_vel;
  m_cmd_pos = m_eff;

}

inline
void JointClaimedResource::callback(const sensor_msgs::JointStateConstPtr& msg, const std::string& topic)
{
  ClaimedResource::callback(msg, topic);

  std::lock_guard<std::mutex> lock(m_mutex);
  std::vector<std::string> names = msg->name;
  std::vector<double> pos = msg->position;
  std::vector<double> vel = msg->velocity;
  std::vector<double> eff = msg->effort;

  std::vector<unsigned int> ax_indexes = m_idxes_ax_map[ topic ];
  if ((pos.size() < ax_indexes.size()) || (vel.size() < ax_indexes.size()) || (eff.size() < ax_indexes.size()) || (names.size() < ax_indexes.size()))
  {
    ROS_FATAL("Dimension are wrong: pos %zu, vel %zu, eff %zu, names %zu, expected %zu", pos.size(), vel.size(), eff.size(), names.size(), ax_indexes.size());
    m_topics_received[topic] = false;
    return;
  }

  if (!name_sorting::permutationName(m_resource_names_map.at(topic), names, pos, vel, eff, "JOINT CLAIMED RESOURCE - TOPICS HW INTERFACE"))
  {
    m_topics_received[topic] = false;
    ROS_WARN_THROTTLE(0.1, "[%s] feedback joint states names are wrong!", m_console.c_str());
    return;
  }
  m_topics_received[topic] = true;
  for (unsigned int idx = 0; idx < m_resource_names_map.at(topic).size(); idx++)
  {
    m_pos.at(ax_indexes.at(idx)) = pos.at(idx);
    m_vel.at(ax_indexes.at(idx)) = vel.at(idx);
    m_eff.at(ax_indexes.at(idx)) = eff.at(idx);
  }
}

inline
void JointClaimedResource::init()
{
  cnr_hardware_interface::ClaimedResource<sensor_msgs::JointState>::init();
  for (std::string& joint_name : m_resource_names)
  {
    auto i = &joint_name - &(m_resource_names[0]);

    hardware_interface::JointStateHandle state_handle(joint_name,
        &(m_pos.at(i)),
        &(m_vel.at(i)),
        &(m_eff.at(i)));


    m_js_jh.registerHandle(state_handle);

    m_p_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_pos.at(i))));
    m_v_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_vel.at(i))));
    m_e_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_eff.at(i))));

    m_pve_jh.registerHandle(hardware_interface::PosVelEffJointHandle(state_handle, &(m_cmd_pos.at(i)), &(m_cmd_vel.at(i)), &(m_cmd_eff.at(i))));
    m_ve_jh.registerHandle(hardware_interface::VelEffJointHandle(state_handle, &(m_cmd_vel.at(i)), &(m_cmd_eff.at(i))));
  }

  m_p_jh_active = m_v_jh_active = m_e_jh_active = false;

  if (m_pub_msg.size())
  {
    m_pub_msg.front().reset(new sensor_msgs::JointState());
    m_pub_msg.front()->name         = m_resource_names;
    m_pub_msg.front()->position     = m_pos;
    m_pub_msg.front()->velocity     = m_vel;
    m_pub_msg.front()->effort       = m_eff;
    std::fill(m_pub_msg.front()->effort.begin(), m_pub_msg.front()->effort.end(), 0.0);
  }
}

inline
void JointClaimedResource::shutdown()
{
  if (!m_shutted_down)
  {
    cnr_hardware_interface::ClaimedResource<sensor_msgs::JointState>::shutdown();
  }
}

inline
void JointClaimedResource::setParam(const std::string& ns)
{
  m_robothw_nh.setParam("status/joint_resource/" + ns + "/feedback/name", m_resource_names);
  m_robothw_nh.setParam("status/joint_resource/" + ns + "/feedback/position", m_pos);
  m_robothw_nh.setParam("status/joint_resource/" + ns + "/feedback/velocity", m_vel);
  m_robothw_nh.setParam("status/joint_resource/" + ns + "/feedback/effort", m_eff);
  m_robothw_nh.setParam("status/joint_resource/" + ns + "/command/name", m_resource_names);
  m_robothw_nh.setParam("status/joint_resource/" + ns + "/command/position", m_cmd_pos);
  m_robothw_nh.setParam("status/joint_resource/" + ns + "/command/velocity", m_cmd_vel);
  m_robothw_nh.setParam("status/joint_resource/" + ns + "/command/effort", m_cmd_eff);
}

inline
void JointClaimedResource::write(const ros::Time& time, const ros::Duration& period)
{
  if (m_pub.size() > 0 && m_pub.front())
  {
    if (!m_p_jh_active && !m_v_jh_active && !m_e_jh_active)
    {
      return;
    }

    if (m_p_jh_active)
    {
      m_pub_msg.front()->position = m_cmd_pos;
    }
    else
    {
      m_pub_msg.front()->position.resize(m_nAx);
      std::fill(m_pub_msg.front()->position.begin(), m_pub_msg.front()->position.end(), 0.0);
    }

    if (m_v_jh_active)
      m_pub_msg.front()->velocity = m_cmd_vel;
    else
    {
      m_pub_msg.front()->velocity.resize(m_nAx);
      std::fill(m_pub_msg.front()->velocity.begin(), m_pub_msg.front()->velocity.end(), 0.0);
    }

    if (m_e_jh_active)
      m_pub_msg.front()->effort   = m_cmd_eff;
    else
    {
      m_pub_msg.front()->effort.resize(m_nAx);
      std::fill(m_pub_msg.front()->effort.begin(), m_pub_msg.front()->effort.end(), 0.0);
    }
    m_pub_msg.front()->name = m_resource_names;
    m_pub_msg.front()->header.stamp = ros::Time::now();

    ClaimedResource::write(time, period);
  }
}

inline
bool JointClaimedResource::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  bool p_jh_active, v_jh_active, e_jh_active;
  p_jh_active = v_jh_active = e_jh_active = false;

  for (const hardware_interface::ControllerInfo& controller : start_list)
  {
    for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
    {
      if (!res.hardware_interface.compare("hardware_interface::PositionJointInterface"))
        p_jh_active = true;
      if (!res.hardware_interface.compare("hardware_interface::VelocityJointInterface"))
        v_jh_active = true;
      if (!res.hardware_interface.compare("hardware_interface::EffortJointInterface"))
        e_jh_active = true;
      if (!res.hardware_interface.compare("hardware_interface::VelEffJointInterface"))
      {
        v_jh_active = true;
        e_jh_active = true;
      }
      if (!res.hardware_interface.compare("hardware_interface::PosVelEffJointInterface"))
      {
        p_jh_active = true;
        v_jh_active = true;
        e_jh_active = true;
      }
    }
  }

  m_p_jh_active = p_jh_active;
  m_v_jh_active = v_jh_active;
  m_e_jh_active = e_jh_active;
  return true;
}


/**
 *
 *
 *
 *
 *
 *
 *
 *
 */
inline
AnalogClaimedResource::AnalogClaimedResource(const cnr_hardware_interface::AnalogResource& ar, 
                                             ros::NodeHandle& robothw_nh, 
                                             std::map< std::string, bool> & topics_received)
: cnr_hardware_interface::ClaimedResource<std_msgs::Float64MultiArray>(ar, ar.m_channel_names, robothw_nh, topics_received)
, m_a_h_active( false )
, m_a_sh_active( false )
{
  m_state.resize(ar.m_num_channels, 0);
  m_output.resize(ar.m_num_channels, 0);

}
inline
void AnalogClaimedResource::callback(const std_msgs::Float64MultiArray::ConstPtr& msg, const std::string& topic)
{
  ClaimedResource::callback(msg, topic);

  std::lock_guard<std::mutex> lock(m_mutex);
  if (msg->data.size() != m_state.size())
  {
    ROS_FATAL("Dimension are wrong: msg %zu, state %zu", msg->data.size(), m_state.size());
    m_topics_received[topic] = false;
    return;
  }
  std::vector<std::string> names(m_state.size());
  for (size_t i = 0; i < m_state.size(); i++)
  {
    names[i] = msg->layout.dim[i].label;
  }

  std::vector<double>       values = msg->data;
  std::vector<unsigned int> ax_indexes = m_idxes_ax_map[ topic ];
  if (values.size() < ax_indexes.size())
  {
    ROS_FATAL("Dimension are wrong: val %zu, expected %zu", values.size(), ax_indexes.size());
    m_topics_received[topic] = false;
    return;
  }

  if (!name_sorting::permutationName(m_resource_names_map.at(topic), names, values, "ANALOG CLAIMED RESOURCE - TOPICS HW INTERFACE"))
  {
    m_topics_received[topic] = false;
    ROS_WARN_THROTTLE(0.1, "[%s] feedback joint states names are wrong!", m_console.c_str());
    return;
  }
  m_topics_received[topic] = true;
  for (unsigned int idx = 0; idx < m_resource_names_map.at(topic).size(); idx++)
  {
    m_state.at(ax_indexes.at(idx)) = values.at(idx);
  }
}

inline
void AnalogClaimedResource::init()
{
  cnr_hardware_interface::ClaimedResource<std_msgs::Float64MultiArray>::init();

  for (std::string& channel_name : m_resource_names)
  {
    auto i = &channel_name - &(m_resource_names[0]);

    hardware_interface::AnalogStateHandle state_handle(channel_name, &(m_state.at(i)));

    m_a_sh.registerHandle(state_handle);
    m_a_h.registerHandle(hardware_interface::AnalogHandle(state_handle, &(m_output.at(i))));
  }

  m_a_sh_active = m_a_h_active = false;

  if (m_pub_msg.size())
  {
    m_pub_msg.front().reset(new std_msgs::Float64MultiArray());
    m_pub_msg.front()->data.resize(m_resource_names.size(), 0);
  }
}


inline
void AnalogClaimedResource::shutdown()
{
  cnr_hardware_interface::ClaimedResource<std_msgs::Float64MultiArray>::shutdown();
}

inline
void AnalogClaimedResource::setParam(const std::string& ns)
{
  m_robothw_nh.setParam("status/analog_resource/" + ns + "/feedback/name", m_resource_names);
  m_robothw_nh.setParam("status/analog_resource/" + ns + "/feedback/values", m_state);
  m_robothw_nh.setParam("status/analog_resource/" + ns + "/command/name", m_resource_names);
  m_robothw_nh.setParam("status/analog_resource/" + ns + "/command/values", m_output);
}

inline
void AnalogClaimedResource::write(const ros::Time& time, const ros::Duration& period)
{
  if (m_pub.size() > 0 && m_pub.front())
  {
    m_pub_msg.front()->data = m_output;

    ClaimedResource::write(time, period);
  }
}

inline
bool AnalogClaimedResource::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
                                          const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  bool a_h_active = m_a_h_active;
  bool a_sh_active = m_a_sh_active;
  for (const hardware_interface::ControllerInfo& controller : start_list)
  {
    for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
    {
      a_sh_active = !res.hardware_interface.compare("hardware_interface::AnalogStateInterface");
      a_h_active = !res.hardware_interface.compare("hardware_interface::AnalogInterface");
    }
  }
  m_a_h_active = a_h_active;
  m_a_sh_active = a_sh_active;
  return true;
}


/**
 *
 *
 *
 *
 *
 *
 *
 *
 */
inline
ForceTorqueClaimedResource::ForceTorqueClaimedResource(const cnr_hardware_interface::ForceTorqueResource& wr,
                                                       ros::NodeHandle& robothw_nh,
                                                       std::map< std::string, bool> & topics_received)
: cnr_hardware_interface::ClaimedResource<geometry_msgs::WrenchStamped> (wr, {wr.m_sensor_name}, robothw_nh, topics_received)
, m_w_h_active(false)
, m_w_sh_active(false)

{
  assert(wr.m_subscribed_topics.size() == 1);
  size_t l = __LINE__;
  try
  {
    l = __LINE__;
    m_frame_id = wr.m_frame_id;
    m_state.resize(6, 0);
    l = __LINE__;
    m_output.resize(6, 0);
    l = __LINE__;
  }
  catch (std::exception& e)
  {
    ROS_FATAL("ClaimedResource error %s (at line:%zu)", e.what(), l);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    throw std::runtime_error(("ClaimedResource error"  + std::string(e.what())) .c_str());
  }
  catch (...)
  {
    ROS_FATAL("ClaimedResource error. Unhandled expcetion at line:%zu", l);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    throw std::runtime_error("ClaimedResource error");
  }


}

inline
void ForceTorqueClaimedResource::callback(const geometry_msgs::WrenchStamped::ConstPtr& msg, const std::string& topic)
{
  ClaimedResource::callback(msg, topic);

  std::lock_guard<std::mutex> lock(m_mutex);

  m_topics_received[topic] = true;
  m_state.at(0) = msg->wrench.force.x;
  m_state.at(1) = msg->wrench.force.y;
  m_state.at(2) = msg->wrench.force.z;
  m_state.at(3) = msg->wrench.torque.x;
  m_state.at(4) = msg->wrench.torque.y;
  m_state.at(5) = msg->wrench.torque.z;
}

inline
void ForceTorqueClaimedResource::init()
{
  cnr_hardware_interface::ClaimedResource<geometry_msgs::WrenchStamped>::init();

  hardware_interface::ForceTorqueStateHandle state_handle(m_resource_names.front()
      , m_frame_id
      , &(m_state.at(0))
      , &(m_state.at(3)));

  m_w_sh.registerHandle(state_handle);
  m_w_h.registerHandle(hardware_interface::ForceTorqueHandle(state_handle, &(m_output.at(0)), &(m_output.at(3))));


  if (m_pub_msg.size())
  {
    m_w_sh_active = m_w_h_active = false;
    m_pub_msg.front().reset(new geometry_msgs::WrenchStamped());
  }
}


inline
void ForceTorqueClaimedResource::shutdown()
{
  if (!m_shutted_down)
  {
    cnr_hardware_interface::ClaimedResource<geometry_msgs::WrenchStamped>::shutdown();
  }
}

inline
void ForceTorqueClaimedResource::setParam(const std::string& ns)
{
  m_robothw_nh.setParam("status/analog_resource/" + ns + "/feedback/name", m_resource_names);
  m_robothw_nh.setParam("status/analog_resource/" + ns + "/feedback/values", m_state);
  m_robothw_nh.setParam("status/analog_resource/" + ns + "/command/name", m_resource_names);
  m_robothw_nh.setParam("status/analog_resource/" + ns + "/command/values", m_output);
}

inline
void ForceTorqueClaimedResource::write(const ros::Time& time, const ros::Duration& period)
{
  if (m_pub.size() > 0 && m_pub.front())
  {
    m_pub_msg.front()->wrench.force.x  = m_output.at(0);
    m_pub_msg.front()->wrench.force.y  = m_output.at(1);
    m_pub_msg.front()->wrench.force.z  = m_output.at(2);
    m_pub_msg.front()->wrench.torque.x = m_output.at(3);
    m_pub_msg.front()->wrench.torque.y = m_output.at(4);
    m_pub_msg.front()->wrench.torque.z = m_output.at(5);

    ClaimedResource::write(time, period);
  }
}

inline
bool ForceTorqueClaimedResource::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  bool h_active = m_w_h_active;
  bool sh_active = m_w_sh_active;
  for (const hardware_interface::ControllerInfo& controller : start_list)
  {
    for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
    {
      sh_active = !res.hardware_interface.compare("hardware_interface::ForceTorqueStateHandle");
      h_active  = !res.hardware_interface.compare("hardware_interface::ForceTorqueInterface");
    }
  }
  m_w_h_active = h_active;
  m_w_sh_active = sh_active;
  return true;
}





/**
 *
 *
 *
 *
 *
 *
 *
 *
 */
inline
PoseClaimedResource::PoseClaimedResource(const cnr_hardware_interface::PoseResource& pr, ros::NodeHandle& robothw_nh, std::map< std::string, bool> & topics_received)
: cnr_hardware_interface::ClaimedResource< geometry_msgs::PoseStamped > (pr, {pr.m_frame_id}, robothw_nh, topics_received)
, m_p_h_active(false)
, m_p_sh_active(false)
{
  size_t l = __LINE__;
  try
  {
    l = __LINE__;
    m_state = geometry_msgs::Pose();
    l = __LINE__;
    m_output = geometry_msgs::Pose();
    l = __LINE__;
  }
  catch (std::exception& e)
  {
    ROS_FATAL("ClaimedResource error %s (at line:%zu)", e.what(), l);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    throw std::runtime_error(("ClaimedResource error"  + std::string(e.what())) .c_str());
  }
  catch (...)
  {
    ROS_FATAL("ClaimedResource error. Unhandled expcetion at line:%zu", l);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    throw std::runtime_error("ClaimedResource error");
  }


}

inline
void PoseClaimedResource::callback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& topic)
{
  ClaimedResource::callback(msg, topic);

  std::lock_guard<std::mutex> lock(m_mutex);

  m_topics_received[topic] = true;
  m_state = msg->pose;
}

inline
void PoseClaimedResource::init()
{
  cnr_hardware_interface::ClaimedResource<geometry_msgs::PoseStamped>::init();

  hardware_interface::PoseStateHandle state_handle(m_resource_names.front()
      , &(m_state));

  m_p_sh.registerHandle(state_handle);
  m_p_h.registerHandle(hardware_interface::PoseHandle(state_handle, &(m_output)));

  m_p_sh_active = m_p_h_active = false;

  if (m_pub_msg.size())
  {
    m_pub_msg.front().reset(new geometry_msgs::PoseStamped());
  }
}


inline
void PoseClaimedResource::shutdown()
{
  if (!m_shutted_down)
  {
    cnr_hardware_interface::ClaimedResource<geometry_msgs::PoseStamped>::shutdown();
  }
}

inline
void PoseClaimedResource::setParam(const std::string& ns)
{
  m_robothw_nh.setParam("status/pose_resource/" + ns + "/feedback/name", m_resource_names);
//m_robothw_nh.setParam("status/pose_resource/"+ns+"/feedback/values"  , m_state          );
  m_robothw_nh.setParam("status/pose_resource/" + ns + "/command/name", m_resource_names);
//m_robothw_nh.setParam("status/pose_resource/"+ns+"/command/values"   , m_output         );
}


inline
void PoseClaimedResource::write(const ros::Time& time, const ros::Duration& period)
{
  if (m_pub.size() > 0 && m_pub.front())
  {
    m_pub_msg.front()->header.stamp = ros::Time::now();
    m_pub_msg.front()->pose = m_output;

    ClaimedResource::write(time, period);
  }
}

inline
bool PoseClaimedResource::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  bool h_active = m_p_h_active;
  bool sh_active = m_p_sh_active;
  for (const hardware_interface::ControllerInfo& controller : start_list)
  {
    for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
    {
      sh_active = !res.hardware_interface.compare("hardware_interface::PoseStateHandle");
      h_active  = !res.hardware_interface.compare("hardware_interface::PoseCommandInterface");
    }
  }
  m_p_h_active = h_active;
  m_p_sh_active = sh_active;
  return true;
}









/**
 *
 *
 *
 *
 *
 *
 *
 *
 */
inline
TwistClaimedResource::TwistClaimedResource(const cnr_hardware_interface::TwistResource& pr, ros::NodeHandle& robothw_nh, std::map< std::string, bool> & topics_received)
  : cnr_hardware_interface::ClaimedResource< geometry_msgs::TwistStamped > (pr, pr.m_frames_id, robothw_nh, topics_received)
#define ll l = __LINE__;
{
  size_t l = __LINE__;
  try
  {
    assert(pr.m_subscribed_topics.size() == pr.m_frames_id.size());
    m_state.resize(pr.m_subscribed_topics.size(), geometry_msgs::TwistStamped());
    ll;
    for (size_t i = 0; i < m_state.size(); i++)
    {
      m_state.at(i).header.frame_id = pr.m_frames_id.at(i);
    }

    m_output.resize(pr.m_published_topics.size(), geometry_msgs::TwistStamped());
    ll;
    for (size_t i = 0; i < m_output.size(); i++)
    {
      m_output.at(i).header.frame_id = pr.m_frames_id.at(i);
    }

    m_t_sh.resize(m_state.size());    //interface for reading joint state
    m_t_h.resize(m_output.size());     //interface for reading joint state

    m_t_sh_active.resize(m_state.size(), false);
    m_t_h_active .resize(m_output.size(), false);
  }
  catch (std::exception& e)
  {
    ROS_FATAL("ClaimedResource error %s (at line:%zu)", e.what(), l);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    throw std::runtime_error(("ClaimedResource error"  + std::string(e.what())) .c_str());
  }
  catch (...)
  {
    ROS_FATAL("ClaimedResource error. Unhandled expcetion at line:%zu", l);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    throw std::runtime_error("ClaimedResource error");
  }
}

inline
void TwistClaimedResource::callback(const geometry_msgs::TwistStamped::ConstPtr& msg, const std::string& topic)
{
  ClaimedResource::callback(msg, topic);

  std::lock_guard<std::mutex> lock(m_mutex);

  m_topics_received[topic] = true;
  for (geometry_msgs::TwistStamped & state : m_state)
  {
    if (state.header.frame_id == msg->header.frame_id)
    {
      state.twist = msg->twist;
      state.header.stamp = ros::Time::now();
    }
  }
}

inline
void TwistClaimedResource::init()
{
  cnr_hardware_interface::ClaimedResource<geometry_msgs::TwistStamped>::init();
  for (size_t i = 0; i < m_state.size(); i++)
  {
    hardware_interface::TwistStateHandle state_handle(m_resource_names.at(i)
        , &(m_state.at(i).twist));

    m_t_sh.at(i).registerHandle(state_handle);
    m_t_h.at(i).registerHandle(hardware_interface::TwistHandle(state_handle, &(m_output.at(i).twist)));

    m_t_sh_active.at(i) = m_t_h_active.at(i) = false;
  }

  for (size_t i = 0; i < m_pub_msg.size(); i++)
  {
    m_pub_msg.at(i).reset(new geometry_msgs::TwistStamped());
  }
}


inline
void TwistClaimedResource::shutdown()
{
  if (!m_shutted_down)
  {
    cnr_hardware_interface::ClaimedResource<geometry_msgs::TwistStamped>::shutdown();
  }
}

inline
void TwistClaimedResource::setParam(const std::string& ns)
{
  m_robothw_nh.setParam("status/twist_resource/" + ns + "/feedback/name", m_resource_names);
//m_robothw_nh.setParam("status/twist_resource/"+ns+"/feedback/values"  , m_state          );
  m_robothw_nh.setParam("status/twist_resource/" + ns + "/command/name", m_resource_names);
//m_robothw_nh.setParam("status/twist_resource/"+ns+"/command/values"   , m_output         );
}


inline
void TwistClaimedResource::write(const ros::Time& time, const ros::Duration& period)
{
  if (m_pub.size() > 0)
  {
    for (size_t i = 0; i < m_pub.size(); i++)
    {
      m_pub_msg.at(i)->header.stamp     = ros::Time::now();
      m_pub_msg.at(i)->header.frame_id  = m_output.at(i).header.frame_id;
      m_pub_msg.at(i)->twist            = m_output.at(i).twist;
    }

    ClaimedResource::write(time, period);
  }
}

inline
bool TwistClaimedResource::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  for (size_t i = 0; i < m_resource_names.size(); i++)
  {
    bool h_active = m_t_h_active.at(i);
    bool sh_active = m_t_sh_active.at(i);
    for (const hardware_interface::ControllerInfo& controller : start_list)
    {
      for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
      {
        sh_active = !res.hardware_interface.compare("hardware_interface::TwistStateHandle");
        h_active  = !res.hardware_interface.compare("hardware_interface::TwistCommandInterface");
      }
    }
    m_t_h_active.at(i) = h_active;
    m_t_sh_active.at(i) = sh_active;
  }
  return true;
}

}  // namespace cnr_hardware_interface

#endif  // CNR_HARDWARE_INTERFACE_CNR_TOPICS_ROBOT_HW_H
