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
      CNR_FATAL(*m_logger, getPrivateNodeHandle().getNamespace() + "/joint_names' does not exist");
      CNR_FATAL(*m_logger, "ERROR DURING STARTING HARDWARE INTERFACE '" << getPrivateNodeHandle().getNamespace() << "'");
      CNR_RETURN_FALSE(*m_logger);
    }
    CNR_DEBUG(*m_logger, "Create the TopicRobotHW (joint names: " << joint_names.size() << ")");
    m_hw.reset(new cnr_hardware_interface::TopicRobotHW(/*joint_names*/));
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
