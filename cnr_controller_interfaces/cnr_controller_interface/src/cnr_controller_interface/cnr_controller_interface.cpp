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

#include <cnr_controller_interface/utils/cnr_handles_utils.h>
#include <cnr_controller_interface/cnr_controller_interface.h>

namespace cnr_controller_interface
{

const KinematicStatus * const getPtr( const KinematicStatusConstPtr& in)
{
  return in.get();
}

const KinematicStatus * const getPtr( const KinematicStatus& in)
{
  return &in;
}


KinematicStatus* getPtr(KinematicStatusPtr& in)
{
  return in.get();
}

KinematicStatus* getPtr( KinematicStatus& in)
{
  return &in;
}

#define RETURN_FALSE(X)\
  return false;

#define CHECK(ks, hi)\
  if(!ks) return false;\
  if(!hi) return false;

#define FOR_EACH_JOINT(iAx, hi, ks)\
  try\
  {\
    std::vector<std::string> names = hi->getNames();\
    for(iAx=0; iAx< ks->joint_names.size(); iAx++ )\
    {\
      auto it = std::find(names.begin(), names.end(), ks->get_joint_name(iAx));\
      if( it == names.end() )\
        return false;\


#define END_FOR_EACH_JOINT( )\
     }\
  }\
  catch(std::exception& e)\
  {\
    std::cerr << __FILE__ << ":" << __LINE__ << " " << e.what() << std::endl;\
  }\
  catch(...)\
  {\
    std::cerr << __FILE__ << ":" << __LINE__ << " Unhandled exception" << std::endl;\
  }\

/**
 * JointStateInterface
 */

bool get_from_hw(hardware_interface::JointStateInterface* const hi, KinematicStatus* ks)
{
  CHECK(ks, hi)

  size_t iAx=0;
  FOR_EACH_JOINT(iAx, hi, ks)
    ks->q(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getPosition();
    ks->qd(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getVelocity();
    ks->qdd(iAx) = 0.0;
    ks->effort(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getEffort();
  END_FOR_EACH_JOINT()
  return true;
}



/**
 * hardware_interface::VelEffJointInterface
 */

bool get_from_hw(hardware_interface::VelEffJointInterface* hi, KinematicStatus* ks)
{
  size_t iAx=0;
  CHECK(ks, hi)
  FOR_EACH_JOINT(iAx, hi, ks)
      ks->q(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getPosition();
      ks->qd(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getVelocity();
      ks->qdd(iAx) = 0.0;
      ks->effort(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getEffort();
  END_FOR_EACH_JOINT()
  return true;
}

bool set_to_hw(const KinematicStatus* const ks, hardware_interface::VelEffJointInterface* hi)
{
  size_t iAx=0;
  CHECK(ks, hi)
  FOR_EACH_JOINT(iAx, hi, ks)
    hi->getHandle(ks->get_joint_name(iAx)).setCommandVelocity(ks->qd(iAx) );
    hi->getHandle(ks->get_joint_name(iAx)).setCommandEffort(ks->effort(iAx) );
  END_FOR_EACH_JOINT()
  return true;
}



/**
 * PosVelEffJointInterface
 */

bool get_from_hw(hardware_interface::PosVelEffJointInterface* const hi, KinematicStatus* ks)
{
  size_t iAx=0;
  CHECK(ks, hi)
  FOR_EACH_JOINT(iAx, hi, ks)
      ks->q(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getPosition();
      ks->qd(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getVelocity();
      ks->qdd(iAx) = 0.0;
      ks->effort(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getEffort();
  END_FOR_EACH_JOINT()
  return true;
}


bool set_to_hw(const KinematicStatus* const ks, hardware_interface::PosVelEffJointInterface* hi)
{
  size_t iAx=0;
  CHECK(ks, hi)
  FOR_EACH_JOINT(iAx, hi, ks)
    hi->getHandle(ks->get_joint_name(iAx)).setCommandPosition(ks->q(iAx) );
    hi->getHandle(ks->get_joint_name(iAx)).setCommandVelocity(ks->qd(iAx) );
    hi->getHandle(ks->get_joint_name(iAx)).setCommandEffort  (ks->effort(iAx) );
  END_FOR_EACH_JOINT()
  return true;
}


/**
 * JointCommandInterface
 */
bool get_from_hw(hardware_interface::JointCommandInterface* const hi, KinematicStatus* ks)
{
  size_t iAx=0;
  CHECK(ks, hi)
  FOR_EACH_JOINT(iAx, hi, ks)
      ks->q(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getPosition();
      ks->qd(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getVelocity();
      ks->qdd(iAx) = 0.0;
      ks->effort(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getEffort();
  END_FOR_EACH_JOINT()
  return true;
}

bool set_to_hw(const KinematicStatus* const ks, hardware_interface::JointCommandInterface* hi)
{
  size_t iAx=0;
  CHECK(ks, hi)
  FOR_EACH_JOINT(iAx, hi, ks)
    hi->getHandle(ks->get_joint_name(iAx)).setCommand(ks->q(iAx) );
  END_FOR_EACH_JOINT()
  return true;
}


/**
 * @brief EffortJointInterface
 */
bool get_from_hw(hardware_interface::EffortJointInterface* const hi, KinematicStatus* ks)
{
  size_t iAx=0;
  CHECK(ks, hi)
  FOR_EACH_JOINT(iAx, hi, ks)
      ks->q(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getPosition();
      ks->qd(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getVelocity();
      ks->qdd(iAx) = 0.0;
      ks->effort(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getEffort();
  END_FOR_EACH_JOINT()
  return true;
}

bool set_to_hw(const KinematicStatus* const ks, hardware_interface::EffortJointInterface* hi)
{
  size_t iAx=0;
  CHECK(ks, hi)
  FOR_EACH_JOINT(iAx, hi, ks)
    hi->getHandle(ks->get_joint_name(iAx)).setCommand(ks->effort(iAx) );
  END_FOR_EACH_JOINT()
  return true;
}

/**
 * @brief VelocityJointInterface
 */
bool get_from_hw(hardware_interface::VelocityJointInterface* const hi, KinematicStatus* ks)
{
  size_t iAx=0;
  CHECK(ks, hi)
  FOR_EACH_JOINT(iAx, hi, ks)
      ks->q(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getPosition();
      ks->qd(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getVelocity();
      ks->qdd(iAx) = 0.0;
      ks->effort(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getEffort();
  END_FOR_EACH_JOINT()
  return true;
}

bool set_to_hw(const KinematicStatus* const ks, hardware_interface::VelocityJointInterface* hi)
{
  size_t iAx=0;
  CHECK(ks, hi)
  FOR_EACH_JOINT(iAx, hi, ks)
    hi->getHandle(ks->get_joint_name(iAx)).setCommand(ks->qd(iAx) );
  END_FOR_EACH_JOINT()
  return true;
}

/**
 * @brief PositionJointInterface
 */
bool get_from_hw(hardware_interface::PositionJointInterface* const hi, KinematicStatus* ks)
{
  size_t iAx=0;
  CHECK(ks, hi)
  FOR_EACH_JOINT(iAx, hi, ks)
      ks->q(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getPosition();
      ks->qd(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getVelocity();
      ks->qdd(iAx) = 0.0;
      ks->effort(iAx) = hi->getHandle(ks->get_joint_name(iAx)).getEffort();
  END_FOR_EACH_JOINT()
  return true;
}

bool set_to_hw(const KinematicStatus* const ks, hardware_interface::PositionJointInterface* hi)
{
  size_t iAx=0;
  CHECK(ks, hi)
  FOR_EACH_JOINT(iAx, hi, ks)
    hi->getHandle(ks->get_joint_name(iAx)).setCommand(ks->q(iAx) );
  END_FOR_EACH_JOINT()
  return true;
}

std::vector<std::string> get_names(const std::vector< controller_manager_msgs::ControllerState >& controllers)
{
  std::vector<std::string> ret;
  for (auto & ctrl : controllers) ret.push_back(ctrl.name);
  return ret;
}
//============


//============ FUNCTIONS TO DEFINE THE PARAMETERS WHERE THE CTRL STATUS IS LOADED
std::string ctrl_list_param(const std::string& hw_name)
{
  return "/" + hw_name + "/status/controllers_list";
}
std::string last_status_param(const std::string& hw_name, const std::string& ctrl_name)
{
  return "/" + hw_name + "/status/controllers/" + ctrl_name + "/last_status";
}

std::string status_param(const std::string& hw_name, const std::string& ctrl_name)
{
  return "/" + hw_name + "/status/controllers/" + ctrl_name + "/status";
}
bool get_state(const std::string& hw_name,
               const std::string& ctrl_name,
               std::string& status,
               std::string& error,
               const ros::Duration& watchdog)
{
  ros::Time st = ros::Time::now();
  const std::string p = last_status_param(hw_name, ctrl_name);
  while (ros::ok())
  {
    if (ros::param::get(p, status))
    {
      break;
    }

    if((watchdog.toSec() > 0) && (ros::Time::now() > (st + watchdog)))
    {
      error += "Timeout. Param " + p + " does not exist";
      return false;
    }
    ros::Duration(0.001).sleep();
  }
  return true;
}

bool check_state(const std::string& hw_name,
                 const std::string& ctrl_name,
                 const std::string& status,
                 std::string& error,
                 const ros::Duration& watchdog)
{
  bool ok = false;
  ros::Time st = ros::Time::now();
  std::string actual_status;
  const std::string p = last_status_param(hw_name, ctrl_name);
  ros::Time n = ros::Time::now();
  while (ros::ok())
  {
    if (ros::param::get(p, actual_status))
    {
      if( actual_status == status)
      {
        ok = true;
        break;
      }
    }

    if((watchdog.toSec() > 0) && (ros::Time::now() > (st + watchdog)))
    {
      error += "Timeout.";
      ok = false;
      break;
    }
    ros::Duration(0.001).sleep();
  }
  ros::Time a = ros::Time::now();
  if(!ok)
  {
    error += "Failed while checking '" + hw_name + "/" + ctrl_name + "' state. ";
    error += "The state is " + actual_status + " while '" + status + "' is expected ";
    error +="(transition waited for " + std::to_string((a-n).toSec()) + "s, ";
    error +="watchdog: "+ std::to_string(watchdog.toSec())+ "s)";
    return false;
  }
  return true;
}
//============



void ControllerDiagnostic::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat, int level)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();

  stat.hardware_id = m_hw_name;
  stat.name        = "Ctrl ["
                   + ( level == (int)diagnostic_msgs::DiagnosticStatus::OK  ? std::string("Info")
                     : level == (int)diagnostic_msgs::DiagnosticStatus::WARN ? std::string("Warn")
                     : std::string("Error") )
                   +"]";

  bool something_to_add = false;
  for (  const diagnostic_msgs::DiagnosticStatus & s : m_diagnostic.status )
  {
    something_to_add |= static_cast<int>( s.level ) == level;
  }
  if ( something_to_add )
  {
    stat.level       = level == (int)diagnostic_msgs::DiagnosticStatus::OK ? diagnostic_msgs::DiagnosticStatus::OK
                     : level == (int)diagnostic_msgs::DiagnosticStatus::WARN ? diagnostic_msgs::DiagnosticStatus::WARN
                     : level == (int)diagnostic_msgs::DiagnosticStatus::ERROR ? diagnostic_msgs::DiagnosticStatus::ERROR
                     : diagnostic_msgs::DiagnosticStatus::STALE;

    stat.summary(stat.level, "Log of the status at ["
         + boost::posix_time::to_iso_string(my_posix_time) + "]");

    for ( const diagnostic_msgs::DiagnosticStatus & s : m_diagnostic.status )
    {
      diagnostic_msgs::KeyValue k;
      k.key = s.name;
      k.value = s.message;
      stat.add(k.key, k.value);
    }
    m_diagnostic.status.erase(
        std::remove_if(
            m_diagnostic.status.begin(),
            m_diagnostic.status.end(),
            [&](diagnostic_msgs::DiagnosticStatus const & p) { return p.level == level; }
        ),
        m_diagnostic.status.end()
    );
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "None Error in the queue ["
         + boost::posix_time::to_iso_string(my_posix_time) + "]");
  }
}

void ControllerDiagnostic::diagnosticsInfo(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  ControllerDiagnostic::diagnostics(stat,diagnostic_msgs::DiagnosticStatus::OK);
}

void ControllerDiagnostic::diagnosticsWarn(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  ControllerDiagnostic::diagnostics(stat,diagnostic_msgs::DiagnosticStatus::WARN);
}

void ControllerDiagnostic::diagnosticsError(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  ControllerDiagnostic::diagnostics(stat,diagnostic_msgs::DiagnosticStatus::ERROR);
}

void ControllerDiagnostic::diagnosticsPerformance(diagnostic_updater::DiagnosticStatusWrapper &stat)
{

  boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
  std::lock_guard<std::mutex> lock(m_mutex);
  stat.hardware_id = m_hw_name;
  stat.level       = diagnostic_msgs::DiagnosticStatus::OK;
  stat.name        = "Ctrl";
  stat.message     = "Cycle Time Statistics [" + boost::posix_time::to_iso_string(my_posix_time) + "]";
  diagnostic_msgs::KeyValue k;
  k.key = m_ctrl_name + " Update [s]";
  k.value = to_string_fix(m_time_span_tracker->getMean())
          + std::string(" [ ") + to_string_fix(m_time_span_tracker->getMin()) + " - "
          + to_string_fix(m_time_span_tracker->getMax()) + std::string(" ] ")
          + std::string("Missed: ") + std::to_string(m_time_span_tracker->getMissedCycles())
          + std::string("/") + std::to_string(m_time_span_tracker->getTotalCycles());

  stat.add(k.key, k.value);
}




}
