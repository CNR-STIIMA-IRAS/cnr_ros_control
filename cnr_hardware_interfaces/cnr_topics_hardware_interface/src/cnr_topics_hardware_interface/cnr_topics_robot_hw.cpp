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

#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <name_sorting/name_sorting.h>


#include <cnr_topics_hardware_interface/cnr_topics_robot_hw.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr_hardware_interface::TopicsRobotHW, cnr_hardware_interface::RobotHW)



#define CNR_FATAL_RETURN( MSG )\
      CNR_RETURN_FALSE(*m_logger, "ERROR DURING STARTING HARDWARE INTERFACE ' "+m_robothw_nh.getNamespace()+" ':" + std::string( MSG ) );\
      std::cout.flush();\


#define WARNING( MSG )\
      CNR_DEBUG(*m_logger, "[ "+ m_robothw_nh.getNamespace() + " ] RobotHW Nodelet: " + std::string( MSG ) );

static size_t line = __LINE__;
#define __LL__ line = __LINE__;


namespace cnr_hardware_interface
{

void setParam(TopicsRobotHW* hw, const std::string& ns)
{
  if (hw->m_resources.count(JOINT_RESOURCE)) hw->m_joint_resource              ->setParam(ns);
  if (hw->m_resources.count(ANALOG_RESOURCE)) hw->m_analog_resource             ->setParam(ns);
  if (hw->m_resources.count(WRENCH_RESOURCE)) hw->m_force_torque_sensor_resource->setParam(ns);
  if (hw->m_resources.count(POSE_RESOURCE)) hw->m_pose_resource               ->setParam(ns);
  if (hw->m_resources.count(TWIST_RESOURCE)) hw->m_twist_resource              ->setParam(ns);
}

std::vector<std::string> getResourceNames(const std::map< cnr_hardware_interface::RESOURCE_ID, std::shared_ptr< cnr_hardware_interface::Resource > >& resources)
{
  std::vector<std::string> ret;
  for (auto const & resource : resources)
  {
    ret.push_back(RESOURCES().at(resource.first));
  }
  return ret;
}


TopicsRobotHW::TopicsRobotHW()
{
  m_set_status_param = boost::bind(setParam, this, _1);
}

bool TopicsRobotHW::doInit()
#define INIT_RESOURCE( RES, res_var, RES_TYPE, CLAIMED_RES_TYPE)\
      if( m_resources.count( RES ) )\
      {\
        CNR_INFO(*m_logger, "[ " << m_robot_name << " ] Create '" <<  #RES << "' Claimed Resource");\
        std::shared_ptr< cnr_hardware_interface::Resource > p  = m_resources.at( RES );\
        std::shared_ptr< cnr_hardware_interface::RES_TYPE > pp = std::static_pointer_cast<cnr_hardware_interface::RES_TYPE>( p );\
        res_var.reset( new CLAIMED_RES_TYPE( *pp, m_robothw_nh, this->m_topics_subscribed ) );\
        CNR_INFO(*m_logger, "[ " << m_robot_name << " ] Initializing '" << #RES << "' Claimed Resource");\
        init##CLAIMED_RES_TYPE( );\
        CNR_INFO(*m_logger, "[ " << m_robot_name << " ] Claimed Resource '"<< #RES << "' succesfully initialized");\
      }

{
  CNR_TRACE_START(*m_logger);
  try
  {
    std::vector<std::string> resources;
    if (!m_robothw_nh.getParam("resources", resources))
    {
      CNR_FATAL_RETURN(m_robothw_nh.getNamespace() + "/resources' does not exist");
    }

    int maximum_missing_cycles;
    if (!m_robothw_nh.getParam("maximum_missing_cycles", maximum_missing_cycles))
    {
      CNR_FATAL_RETURN(m_robothw_nh.getNamespace() + "/maximum_missing_cycles does not exist");
    }

    for (auto const & resource : resources)
    {
      std::map< cnr_hardware_interface::RESOURCE_ID, std::string >::const_iterator it = cnr_hardware_interface::RESOURCES().begin();

      for (it = cnr_hardware_interface::RESOURCES().begin() ; it !=  cnr_hardware_interface::RESOURCES().end(); ++it)
      {
        if (it->second == resource)
        {
          break;
        }
      }

      if (it != cnr_hardware_interface::RESOURCES().end())
      {
        CNR_DEBUG(*m_logger, "Reading param for resource: '" << it->second);
        std::string ns = m_robothw_nh.getNamespace() + "/" + it->second;

        std::shared_ptr< cnr_hardware_interface::Resource > claimed_resource;
        switch (it->first)
        {

        case cnr_hardware_interface::JOINT_RESOURCE:
        {
          std::shared_ptr< cnr_hardware_interface::JointResource > jr(new cnr_hardware_interface::JointResource());
          std::vector<std::string>  joint_names;
          ROS_DEBUG_STREAM(ns << " Joint Names:");
          if (!m_robothw_nh.getParam(it->second + "/joint_names", joint_names))
          {
            CNR_FATAL_RETURN(ns + "/joint_names does not exist");
          }

          jr->m_joint_names = joint_names;
          if (joint_names.size() == 0)
          {
            CNR_FATAL_RETURN(ns + "/joint_names has size zero");
          }
          for (auto const & jn :  jr->m_joint_names)
            ROS_DEBUG_STREAM(ns << " - " << jn);
          claimed_resource = jr;

        }
        break;
        case cnr_hardware_interface::WRENCH_RESOURCE:
        {
          std::shared_ptr< cnr_hardware_interface::ForceTorqueResource > wr(new cnr_hardware_interface::ForceTorqueResource());
          std::string sensor_name;
          if (!m_robothw_nh.getParam(it->second + "/sensor_name", sensor_name))
          {
            CNR_FATAL_RETURN(ns + "/sensor_name does not exist");
          }
          std::string frame_id;
          if (!m_robothw_nh.getParam(it->second + "/frame_id", frame_id))
          {
            CNR_FATAL_RETURN(ns + "/frame_id does not exist");
          }

          wr->m_sensor_name = sensor_name;
          wr->m_frame_id    = frame_id;
          claimed_resource = wr;
        }
        break;
        case cnr_hardware_interface::ANALOG_RESOURCE:
        {
          std::shared_ptr< cnr_hardware_interface::AnalogResource > ar(new cnr_hardware_interface::AnalogResource());
          std::vector< std::string > channel_names;
          if (!m_robothw_nh.getParam(it->second + "/channel_names", channel_names))
          {
            CNR_FATAL_RETURN(ns + "/channel_names does not exist");
          }
          ar->m_channel_names = channel_names;
          claimed_resource = ar;
        }
        break;
        case cnr_hardware_interface::POSE_RESOURCE:
        {
          std::shared_ptr< cnr_hardware_interface::PoseResource > pr(new cnr_hardware_interface::PoseResource());
          std::string frame_id;
          if (!m_robothw_nh.getParam(it->second + "/frame_id", frame_id))
          {
            CNR_FATAL_RETURN(ns + "/frame_id does not exist");
          }
          pr->m_frame_id    = frame_id;
          claimed_resource = pr;
        }
        break;
        case cnr_hardware_interface::TWIST_RESOURCE:
        {
          std::shared_ptr< cnr_hardware_interface::TwistResource > pr(new cnr_hardware_interface::TwistResource());
          std::vector< std::string > frames_id;
          if (!m_robothw_nh.getParam(it->second + "/frames_id", frames_id))
          {
            CNR_FATAL_RETURN(ns + "/frames_id does not exist");
          }
          pr->m_frames_id = frames_id;
          if (pr->m_frames_id.size() == 0)
          {
            CNR_FATAL_RETURN(ns + "/frames_id has size zero");
          }
          for (auto const & fr :  pr->m_frames_id)
            CNR_DEBUG(*m_logger, ns << " - " << fr);
          claimed_resource = pr;
        }
        break;
        }

        //********************************
        //
        //********************************
        std::vector< std::string > published_topics;

        if (!m_robothw_nh.getParam(it->second + "/published_topics", published_topics))
        {
          std::string published_topic = "N/A";
          if (!m_robothw_nh.getParam(it->second + "/published_topic", published_topic))
          {
            WARNING(it->second + "/published_topic does not exist");
            WARNING(it->second + "/published_topics does not exist");
            published_topics.clear();
          }
          else
          {
            published_topics = {published_topic};
          }
        }
        claimed_resource->m_published_topics = published_topics;
        //********************************
        //
        //********************************
        std::vector< std::string > subscribed_topics;
        if (!m_robothw_nh.getParam(ns + "/subscribed_topics", subscribed_topics))
        {
          std::string subscribed_topic = "N/A";
          if (!m_robothw_nh.getParam(ns + "/subscribed_topic", subscribed_topic))
          {
            subscribed_topics.clear();
          }
          else
          {
            subscribed_topics = { subscribed_topic };
          }
        }

        if (subscribed_topics.size() == 0)
        {
          WARNING("There is not any subsribed topic, The params '" + it->second + "/subscribed_topic(s)' do not exist");
        }
        else
        {
          CNR_DEBUG(*m_logger, "Subscribed topics: ");
          for (auto const & s : subscribed_topics)
          {
            CNR_DEBUG(*m_logger, " - " << s);
          }
        }

        claimed_resource->m_subscribed_topics =  subscribed_topics;

        double feedback_joint_state_timeout_s = 1e-3;
        if (!m_robothw_nh.getParam(it->second + "/feedback_joint_state_timeout_s", published_topics))
        {
          WARNING(it->second + +"/feedback_joint_state_timeout_s does not exist");
        }
        claimed_resource->m_feedback_joint_state_timeout_s = feedback_joint_state_timeout_s;


        m_resources[ it->first ] = claimed_resource;
      }
      else
      {
        CNR_WARN(*m_logger,  m_robothw_nh.getNamespace() << "/resources/" <<  resource << "'  is not supported");
        CNR_WARN(*m_logger,  " Available Resource: " << cnr_hardware_interface::AVAILABLE_RESOURCES());
      }

    }
    if (m_resources.size() == 0)
    {
      CNR_FATAL_RETURN("No claimed resources. ?!?!?");
    }

    CNR_DEBUG(*m_logger, "Create the TopicsRobotHW (claimed resources: " << m_resources.size() << ", max missing messages: " << maximum_missing_cycles);


    m_resource_names = getResourceNames(m_resources);
    m_joint_resource = nullptr;
    m_missing_messages = 0;
    m_max_missing_messages = maximum_missing_cycles;

    INIT_RESOURCE(JOINT_RESOURCE, m_joint_resource, JointResource, JointClaimedResource);
    INIT_RESOURCE(ANALOG_RESOURCE, m_analog_resource, AnalogResource, AnalogClaimedResource);
    INIT_RESOURCE(WRENCH_RESOURCE, m_force_torque_sensor_resource, ForceTorqueResource, ForceTorqueClaimedResource);
    INIT_RESOURCE(POSE_RESOURCE, m_pose_resource, PoseResource, PoseClaimedResource);
    INIT_RESOURCE(TWIST_RESOURCE, m_twist_resource, TwistResource, TwistClaimedResource);

    CNR_INFO(*m_logger, "[ " << m_robot_name << " ] Ok, TopicsRobotHW initialized");

  }
  catch (std::exception& e)
  {
    ROS_FATAL("TopicsRobotHW error %s (at line:%zu)", e.what(), line);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    return false;
  }
  catch (...)
  {
    ROS_FATAL("TopicsRobotHW error. Unhandled expcetion at line:%zu", line);
    std::cout.flush();
    fflush(stdout);
    fflush(stderr);
    return false;
  }

  return true;

#undef INIT_RESOURCE
}

bool TopicsRobotHW::doRead(const ros::Time& time, const ros::Duration& period)
{
  if (!topicsReceived())
  {
    m_missing_messages++;
  }
  else
  {
    m_missing_messages = 0;
  }

  resetTopicsReceived();

  if (m_missing_messages > m_max_missing_messages)
  {
    if (getStatus() == cnr_hardware_interface::RUNNING)
    {
      add_diagnostic_message("ERROR", "maximum_missing_cycles " + std::to_string(m_missing_messages) + "s ", {{"read", "missing messages"}}, true);
      return false;
    }
  }
  return true;
}

bool TopicsRobotHW::doWrite(const ros::Time& time, const ros::Duration& period)
{
  if (m_resources.count(JOINT_RESOURCE))
  {
    assert(m_joint_resource);
    m_joint_resource->write(time, period);
  }
  if (m_resources.count(ANALOG_RESOURCE))
  {
    assert(m_analog_resource);
    m_analog_resource->write(time, period);
  }
  if (m_resources.count(WRENCH_RESOURCE))
  {
    assert(m_force_torque_sensor_resource);
    m_force_torque_sensor_resource->write(time, period);
  }
  if (m_resources.count(POSE_RESOURCE))
  {
    assert(m_pose_resource);
    m_pose_resource->write(time, period);
  }
  if (m_resources.count(TWIST_RESOURCE))
  {
    assert(m_twist_resource);
    m_twist_resource->write(time, period);
  }
  return true;
}

bool TopicsRobotHW::doPrepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  if (m_resources.count(JOINT_RESOURCE))
  {
    assert(m_joint_resource);
    m_joint_resource->prepareSwitch(start_list, stop_list);
  }
  if (m_resources.count(ANALOG_RESOURCE))
  {
    assert(m_analog_resource);
    m_analog_resource->prepareSwitch(start_list, stop_list);
  }
  if (m_resources.count(WRENCH_RESOURCE))
  {
    assert(m_force_torque_sensor_resource);
    m_force_torque_sensor_resource->prepareSwitch(start_list, stop_list);
  }
  if (m_resources.count(POSE_RESOURCE))
  {
    assert(m_pose_resource);
    m_pose_resource->prepareSwitch(start_list, stop_list);
  }
  if (m_resources.count(TWIST_RESOURCE))
  {
    assert(m_twist_resource);
    m_twist_resource->prepareSwitch(start_list, stop_list);
  }
  return true;
}



bool TopicsRobotHW::doCheckForConflict(const std::list< hardware_interface::ControllerInfo >& info)
#define CHECK_RESOURCE( RES, res_var )\
if( m_resources.count( RES ) )\
{\
  assert( res_var );\
  if( res_var->checkForConflict(info) )\
  {\
    add_diagnostic_message("ERROR", "The resource '" + std::string( #RES ) + "' is in conflict with another controller", {{"Transition","switching"}} , true );\
  }\
}

{

  CHECK_RESOURCE(JOINT_RESOURCE, m_joint_resource);
  CHECK_RESOURCE(ANALOG_RESOURCE, m_analog_resource);
  CHECK_RESOURCE(WRENCH_RESOURCE, m_force_torque_sensor_resource);
  CHECK_RESOURCE(POSE_RESOURCE, m_pose_resource);
  CHECK_RESOURCE(TWIST_RESOURCE, m_twist_resource);


  return false;

#undef CHECK_RESOURCE
}

bool TopicsRobotHW::doShutdown()
#define SHUTDOWN_RESOURCE( RES, res_var )\
  if( m_resources.count( RES ) )\
  {\
    assert( res_var );\
    res_var->shutdown();\
  }\

{
  if (!m_shutted_down)
  {
    SHUTDOWN_RESOURCE(JOINT_RESOURCE, m_joint_resource);
    SHUTDOWN_RESOURCE(ANALOG_RESOURCE, m_analog_resource);
    SHUTDOWN_RESOURCE(WRENCH_RESOURCE, m_force_torque_sensor_resource);
    SHUTDOWN_RESOURCE(POSE_RESOURCE, m_pose_resource);
    SHUTDOWN_RESOURCE(TWIST_RESOURCE, m_twist_resource);
  }
  return true;
}

bool TopicsRobotHW::initJointClaimedResource()
{
  bool ret = true;
  try
  {

    if (m_resources.count(JOINT_RESOURCE))
    {
      assert(m_joint_resource);
      m_joint_resource->init();

      registerInterface(&m_joint_resource->m_js_jh);
      registerInterface(&m_joint_resource->m_p_jh);
      registerInterface(&m_joint_resource->m_v_jh);
      registerInterface(&m_joint_resource->m_e_jh);
      registerInterface(&m_joint_resource->m_pve_jh);
      registerInterface(&m_joint_resource->m_ve_jh);

    }
  }
  catch (std::exception& e)
  {
    ROS_FATAL("Caddbbbbugth a exception: %s", e.what());
    ret = false;
  }
  catch (...)
  {
    ROS_FATAL("Caugth a exception");
    ret = false;
  }


  return ret;
}

bool TopicsRobotHW::initAnalogClaimedResource()
{
  bool ret = true;
  if (m_resources.count(ANALOG_RESOURCE))
  {
    assert(m_analog_resource);
    m_analog_resource->init();

    registerInterface(&m_analog_resource->m_a_h);
    registerInterface(&m_analog_resource->m_a_sh);
  }

  return ret;
}


bool TopicsRobotHW::initForceTorqueClaimedResource()
{
  bool ret = false;
  if (m_resources.count(WRENCH_RESOURCE))
  {
    assert(m_force_torque_sensor_resource);

    m_force_torque_sensor_resource->init();

    registerInterface(&m_force_torque_sensor_resource->m_w_sh);
    registerInterface(&m_force_torque_sensor_resource->m_w_h);

    ret = true;
  }

  return ret;

}



bool TopicsRobotHW::initPoseClaimedResource()
{
  bool ret = false;
  if (m_resources.count(POSE_RESOURCE))
  {
    assert(m_pose_resource);

    m_pose_resource->init();

    registerInterface(&m_pose_resource->m_p_sh);
    registerInterface(&m_pose_resource->m_p_h);

    ret = true;
  }

  return ret;

}


bool TopicsRobotHW::initTwistClaimedResource()
{
  bool ret = false;
  if (m_resources.count(TWIST_RESOURCE))
  {
    assert(m_twist_resource);

    m_twist_resource->init();
    for (size_t i = 0; i < m_twist_resource->m_state.size(); i++)
    {
      registerInterface(&m_twist_resource->m_t_sh.at(i));
      registerInterface(&m_twist_resource->m_t_h .at(i));
    }

    ret = true;
  }

  return ret;

}



}

