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
#include <cnr_topics_hardware_interface/cnr_topics_robot_hw_nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr_hardware_nodelet_interface::TopicsRobotHwNodelet, nodelet::Nodelet)

#define CNR_FATAL_RETURN( MSG )\
      CNR_RETURN_FALSE(*m_logger, "ERROR DURING STARTING HARDWARE INTERFACE ' "+getPrivateNodeHandle().getNamespace()+" ':" + std::string( MSG ) );\
      std::cout.flush();\


#define WARNING( MSG )\
      CNR_DEBUG(*m_logger, "[ "+ getPrivateNodeHandle().getNamespace() + " ] RobotHW Nodelet: " + std::string( MSG ) );



namespace cnr_hardware_nodelet_interface
{
bool TopicsRobotHwNodelet::doOnInit()
{
  CNR_TRACE_START(*m_logger);
  try
  {
    std::vector<std::string> resources;
    if (!getPrivateNodeHandle().getParam("resources", resources))
    {
      CNR_FATAL_RETURN(m_hw_name + "/resources' does not exist");
    }

    int maximum_missing_cycles;
    if (!getPrivateNodeHandle().getParam("maximum_missing_cycles", maximum_missing_cycles))
    {
      CNR_FATAL_RETURN(m_hw_name + "/maximum_missing_cycles does not exist");
    }


    std::map< cnr_hardware_interface::RESOURCE_ID, std::shared_ptr< cnr_hardware_interface::Resource > > claimed_resources;

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
        std::string ns = getPrivateNodeHandle().getNamespace() + "/" + it->second;

        std::shared_ptr< cnr_hardware_interface::Resource > claimed_resource;
        switch (it->first)
        {

        case cnr_hardware_interface::JOINT_RESOURCE:
        {
          std::shared_ptr< cnr_hardware_interface::JointResource > jr(new cnr_hardware_interface::JointResource());
          std::vector<std::string>  joint_names;
          ROS_DEBUG_STREAM(ns << " Joint Names:");
          if (!getPrivateNodeHandle().getParam(it->second + "/joint_names", joint_names))
          {
            CNR_FATAL_RETURN(ns + "/joint_names does not exist");
          }

          jr->m_joint_names = joint_names;

          jr->m_nAx = joint_names.size();
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
          if (!getPrivateNodeHandle().getParam(it->second + "/sensor_name", sensor_name))
          {
            CNR_FATAL_RETURN(ns + "/sensor_name does not exist");
          }
          std::string frame_id;
          if (!getPrivateNodeHandle().getParam(it->second + "/frame_id", frame_id))
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
          if (!getPrivateNodeHandle().getParam(it->second + "/channel_names", channel_names))
          {
            CNR_FATAL_RETURN(ns + "/channel_names does not exist");
          }
          ar->m_channel_names = channel_names;
          ar->m_num_channels = ar->m_channel_names.size();
          claimed_resource = ar;
        }
        break;
        case cnr_hardware_interface::POSE_RESOURCE:
        {

          std::shared_ptr< cnr_hardware_interface::PoseResource > pr(new cnr_hardware_interface::PoseResource());
          std::string frame_id;
          if (!getPrivateNodeHandle().getParam(it->second + "/frame_id", frame_id))
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
          if (!getPrivateNodeHandle().getParam(it->second + "/frames_id", frames_id))
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
        //          std::cout << "HAS PARAM published_topics? "  << ( getPrivateNodeHandle().hasParam( it->second + "/published_topics")  ? "YES" : "NO" ) << std::endl;
        //          std::cout << "HAS PARAM published_topic? "  << ( getPrivateNodeHandle().hasParam( it->second + "/published_topic")  ? "YES" : "NO" ) << std::endl;

        if (!getPrivateNodeHandle().getParam(it->second + "/published_topics", published_topics))
        {
          std::string published_topic = "N/A";
          if (!getPrivateNodeHandle().getParam(it->second + "/published_topic", published_topic))
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
        if (!getPrivateNodeHandle().getParam(ns + "/subscribed_topics", subscribed_topics))
        {
          std::string subscribed_topic = "N/A";
          if (!getPrivateNodeHandle().getParam(ns + "/subscribed_topic", subscribed_topic))
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
        if (!getPrivateNodeHandle().getParam(it->second + "/feedback_joint_state_timeout_s", published_topics))
        {
          WARNING(it->second + +"/feedback_joint_state_timeout_s does not exist");
        }
        claimed_resource->m_feedback_joint_state_timeout_s = feedback_joint_state_timeout_s;


        claimed_resources[ it->first ] = claimed_resource;
      }
      else
      {
        CNR_WARN(*m_logger,  getPrivateNodeHandle().getNamespace() << "/resources/" <<  resource << "'  is not supported");
        CNR_WARN(*m_logger,  " Available Resource: " << cnr_hardware_interface::AVAILABLE_RESOURCES());
      }

    }
    if (claimed_resources.size() == 0)
    {
      CNR_FATAL_RETURN("No claimed resources. ?!?!?");
    }

    CNR_DEBUG(*m_logger, "Create the TopicsRobotHW (claimed resources: " << claimed_resources.size() << ", max missing messages: " << maximum_missing_cycles);
    auto s = getResourceNames(claimed_resources);
    CNR_DEBUG(*m_logger, "Resource Names: " << cnr_controller_interface::to_string(s));

    m_hw.reset(new cnr_hardware_interface::TopicsRobotHW(/*claimed_resources, maximum_missing_cycles*/));
    CNR_DEBUG(*m_logger, "TopicRobotHW Created");
  }
  catch (std::exception& e)
  {
    CNR_FATAL(*m_logger, getPrivateNodeHandle().getNamespace() + " exception in TopicRobotHw constructor. Abort: " << std::string(e.what()));
    CNR_RETURN_FALSE(*m_logger);
  }
  CNR_RETURN_TRUE(*m_logger);
}


}
#undef CNR_FATAL_RETURN
#undef WARNING
