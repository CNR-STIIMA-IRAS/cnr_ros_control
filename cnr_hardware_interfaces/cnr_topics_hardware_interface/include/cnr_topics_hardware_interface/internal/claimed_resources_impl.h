#ifndef CNR_TOPICS_HARDWARE_INTERFACE__CLAIMED_RESOURCES_IMPL__H
#define CNR_TOPICS_HARDWARE_INTERFACE__CLAIMED_RESOURCES_IMPL__H

#include "cnr_topics_hardware_interface/claimed_resources.h"

namespace cnr_hardware_interface
{


template < typename MSG >
ClaimedResource<MSG>::ClaimedResource(const cnr_hardware_interface::Resource& res
                                    , const std::vector<std::string>&         resource_names
                                    , ros::NodeHandle&                        robothw_nh
                                    , std::map< std::string, bool> &          topics_received)
  : m_namespace(robothw_nh.getNamespace())
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
      *sub = robothw_nh.subscribe< MSG >(subscribed_topic, 1, 
                boost::bind(&cnr_hardware_interface::ClaimedResource< MSG >::callback, this, _1, subscribed_topic));
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


template < typename MSG >
ClaimedResource<MSG>:: ~ClaimedResource()
  {
    m_mutex.lock();
    m_mutex.unlock();
  }

template < typename MSG >
void ClaimedResource<MSG>::init()
{

}

template < typename MSG >
void ClaimedResource<MSG>::setParam(const std::string& ns)
{

}

template < typename MSG >
void ClaimedResource<MSG>::write(const ros::Time& time, const ros::Duration& period)
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

template < typename MSG >
void ClaimedResource<MSG>::callback(const typename MSG::ConstPtr& msg, const std::string& topic)
{
  m_msg_counter++;
}

template < typename MSG >
void ClaimedResource<MSG>::shutdown()
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

template < typename MSG >
bool ClaimedResource<MSG>::checkForConflict(const std::list< hardware_interface::ControllerInfo >& info)
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

}  // namespace cnr_hardware_interface

#endif  // CNR_TOPICS_HARDWARE_INTERFACE__CLAIMED_RESOURCES__H