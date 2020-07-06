#ifndef __ITIA_FAKE_HARDWARE_INTERFACE__
#define __ITIA_FAKE_HARDWARE_INTERFACE__

#include <cnr_hardware_interface/cnr_robot_hw.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <mutex>




namespace cnr_hardware_interface
{
  
  class FakeRobotHW: public cnr_hardware_interface::RobotHW
  {
  public:
    FakeRobotHW( );
    ~FakeRobotHW( );

    virtual bool doInit ( );
    virtual bool doWrite(const ros::Time& time, const ros::Duration& period);

    virtual bool doPrepareSwitch    ( const std::list< hardware_interface::ControllerInfo >& start, const std::list< hardware_interface::ControllerInfo >& stop);
    virtual bool doCheckForConflict ( const std::list<hardware_interface::ControllerInfo>& info) ;
    
  protected:

    sensor_msgs::JointState* m_msg;
    void initialJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    hardware_interface::JointStateInterface    m_js_jh; //interface for reading joint state
    hardware_interface::PositionJointInterface m_p_jh; //interface for writing position target
    hardware_interface::VelocityJointInterface m_v_jh; //interface for writing velocity target
    hardware_interface::EffortJointInterface   m_e_jh; //interface for writing effort target
    hardware_interface::PosVelEffJointInterface m_pve_jh;
    hardware_interface::VelEffJointInterface m_ve_jh;
    
    
    bool m_p_jh_active;
    bool m_v_jh_active;
    bool m_e_jh_active;
    
    
    std::vector<double> m_pos; // feedback position
    std::vector<double> m_vel; // feedback velocity
    std::vector<double> m_eff; // feedback effort
    
    std::vector<double> m_cmd_pos; //target position
    std::vector<double> m_cmd_vel; //target velocity
    std::vector<double> m_cmd_eff; //target effort
    
    

    std::mutex m_mutex;
    
    enum status { created, initialized, run, error };
    
    friend void setParam(FakeRobotHW* hw, const std::string& ns);

  };

  void setParam(FakeRobotHW* hw, const std::string& ns);

}
#endif
