# CNR Hardware Interface

The `cnr_hardware_interface` [(readme)](cnr_hardware_interface/cnr_hardware_interface/README.md) is **fully compatible with [ros_control](http://wiki.ros.org/ros_control "ros_control")**. It implements a derived class  `cnr_hardware_interface::RobotHW` that inherits the standard `RobotHW`. The class implements some useful functionalities:

1. It Integrates a [diagnostics](http://wiki.ros.org/diagnostics) module, to track the times of the read/write and update functions
2. It provides a tracking of the state of the class, and it reacts in case of error
3. It provides a `prepareSwitch` function to ease the inherited classes
4. It integrares a [logger module](www.www) that tracks the input and ouptus of the function, and generates log file for the class.

## CNR RobotHW 

The core of the package is built on the standard `RobotHW` from `robot_hw.h` of the `ros_control` architecture. Technically, the standard `RobotHW` can be used togheter the `cnr_ros_control`. Indeed, the class `cnr_hardware_interface::RobotHW` only provides further modules, but it does not change the design of the `RobotHW` class.
ALmost all the inherited functions are split in three sub-methods: `enterFunction`, `doFunction`, `exitFunction`, and this allows to easy track the class usage, and its state.

At [cnr_fake_hardware_interface](../cnr_fake_hardware_interface/README.md) you find an example of how the class `cnr_hardware_interface::RobotHW` can be inherited and customized.

```cpp
class RobotHW: public hardware_interface::RobotHW
{
public:
  RobotHW();
  ~RobotHW();

  // ======================================================= final methods (cannot be overriden by the derived clases)
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) final;
  void read(const ros::Time& time, const ros::Duration& period) final;
  void write(const ros::Time& time, const ros::Duration& period) final;
  bool prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
                     const std::list< hardware_interface::ControllerInfo >& stop_list) final;
  bool checkForConflict(const std::list< hardware_interface::ControllerInfo >& info) const final;
  bool shutdown();
  // ======================================================= End - final methods

  // ======================================================= Method to override inthe derived classes
  virtual bool doInit() { return true; }
  virtual bool doShutdown()  { return true;   }
  virtual bool doRead(const ros::Time& time, const ros::Duration& period) { return true; }
  virtual bool doWrite(const ros::Time& time, const ros::Duration& period)  { return true;  }
  virtual bool doPrepareSwitch(const std::list< hardware_interface::Controller\Info >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list) { return true; }
  virtual bool doCheckForConflict(const std::list< hardware_interface::ControllerInfo >& info) const { return true;  }
  // ======================================================= END - Method to override inthe derived classes


  void setResourceNames(const std::vector<std::string>& resource_names) { m_resource_names = resource_names; }
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);  
  const cnr_hardware_interface::StatusHw&  getStatus() const  {  return m_status;  }
  const std::string&                       getRobotHwNamespace() const  {  return m_robothw_nh.getNamespace();  }
  
protected:
  virtual bool setParamServer(configuration_msgs::SetConfigRequest& req, configuration_msgs::SetConfigResponse& res);
  virtual bool getParamServer(configuration_msgs::GetConfigRequest& req, configuration_msgs::GetConfigResponse& res);

  void addDiagnosticsMessage(const std::string& level, const std::string& summary, const std::map<std::string, std::string>& key_values, const bool verbose = false);
  bool dump_state(const cnr_hardware_interface::StatusHw& status) const;
  bool dump_state() const;

private:
  virtual bool enterInit(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  virtual bool enterShutdown();
  virtual bool enterWrite();
  virtual bool enterPrepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list);
  virtual bool enterCheckForConflict(const std::list< hardware_interface::ControllerInfo >& info) const;

  virtual bool exitInit();
  virtual bool exitShutdown();
  virtual bool exitWrite();
  virtual bool exitPrepareSwitch();
  virtual bool exitCheckForConflict() const{ return true; }

protected:
  std::string                                      m_robot_name;
  ros::NodeHandle                                  m_root_nh;
  ros::NodeHandle                                  m_robothw_nh;
  ros::CallbackQueue                               m_robot_hw_queue;
  std::shared_ptr<cnr_logger::TraceLogger>         m_logger;

  SetStatusParamFcn                                m_set_status_param;

  std::mutex                                       m_mutex;
  mutable diagnostic_msgs::DiagnosticArray         m_diagnostic;
  ros::ServiceServer                               m_get_param;
  ros::ServiceServer                               m_set_param;
  bool                                             m_stop_thread;

  bool                                             m_is_first_read;
  mutable cnr_hardware_interface::StatusHw         m_status;
  mutable std::vector<std::string>                 m_status_history;

  std::list< hardware_interface::ControllerInfo >  m_active_controllers;
  std::vector< std::string >                       m_resource_names;
  bool                                             m_shutted_down;
};

typedef std::shared_ptr<RobotHW> RobotHWSharedPtr;
```


## Dependencies ##

The package provides a .rosinstall file with the dependencies, and the dependencies are configured in the `package.xml` of the packages.

```shell
\$ export ROSINSTALL_FILE=dependencies.rosinstall
\$ mkdir -p ~/catkin_ws/src
\$ cd ~/catkin_ws/src
\$ git clone https://github.com/CNR-STIIMA-IRAS/cnr_hardware_interface
\$ wstool init
\$ wstool merge cnr_ros_control/dependencies.rosinstall
\$ wstool up -v
\$ cd ~/catkin_ws
\$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
## Developer Contact

**Authors:**

- Manuel Beschi (<mailto:manuel.beschi@stiima.cnr.it>)
- Nicola Pedrocchi (<mailto:nicola.pedrocchi@stiima.cnr.it>)  

_Software License Agreement (BSD License)_
_Copyright (c) 2017, National Research Council of Italy, Institute of Industrial Technologies and Automation_
_All rights reserved._
