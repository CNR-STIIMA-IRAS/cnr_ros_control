# Topics hardware interface

The packages implements a Robot HW to subscribe some topics to register resources (joints, force-torque, analog) in a unique Robot HW.
It can be used in the configurations managed by the cnr_controller_manager.

The repository contains the implementation of a hardware interface derived from **cnr_hardware_interface**.

The repository is developed by the Institute of Intelligent Industrial Technologies and Systems for Advanced Manufacturing (STIIMA), of the National Research Council of Italy (CNR).


TopicsRobotHW has the following hardware interfaces:

- hardware_interface::JointStateInterface     
- hardware_interface::PositionJointInterface  
- hardware_interface::VelocityJointInterface  
- hardware_interface::EffortJointInterface    
- hardware_interface::PosVelEffJointInterface 
- hardware_interface::VelEffJointInterface    
- 
## Functionalities and code organization

The package **cnr_topics_hardware_interface** is made the class **TopicsRobotHW**, which implements some additional features wrt to **BasicRobotHW**:

> _checkForConflict_ policy is different from ROS-Control standard.
> Each controller can use more than a hardware_interface for a single joint (for example: position, velocity, effort).
> One controller can control more than one joint.
> A joint can be used only by a controller.

## Parameters

Topics hardware interface requires the following parameters:

```yaml
planner_hw: # name of the hardware interface
  type           : "cnr/control/TopicsRobotHW"
  appenders      : [file, screen]  # logging information: the first one is stored in a file, the second one in the screen
  levels         : [trace, debug]  # trace debug info warn error fatal
  pattern_layout : "[%5p][%d{HH:mm:ss,SSS}][%50M:%04L][%24c] %m%n"
  file_name      : planner_hw  # name of the file (it is saved in ~/.ros/)

  resources: [ joint_resource ] # this TopicsRobotHW manages topic of type: joint
  joint_resource:  # joint handles managed by planner_hw
    joint_names: # name of the joint handles
    - linear_motor_cursor_joint
    - ur5_shoulder_pan_joint
    - ur5_shoulder_lift_joint
    - ur5_elbow_joint
    - ur5_wrist_1_joint
    - ur5_wrist_2_joint
    - ur5_wrist_3_joint
    subscribed_topics : # name of topics sensor_msgs/JointState which contains the state of the joint handle
    - /elmo/joint_states
    - /ur5/joint_states
    - /ur5/joint_states
    - /ur5/joint_states
    - /ur5/joint_states
    - /ur5/joint_states
    - /ur5/joint_states
    published_topics : # name of the topic (type sensor_msgs/JointState) that planner_hw will publish (it has to be only one)
    - /ur5/joint_target

  sampling_period: 0.008
  diagnostic_period: 0.1
  feedback_joint_state_timeout: 20
  maximum_missing_cycles: 1000
  base_link: ur5_base_link  # root of the chain
  tool_link: ur5_tool0      # endeffector of the chain
  robot_description_param: /robot_description   # URDF descriptor
  robot_description_planning_param: /robot_description_planning  

```

## Developer Contact

**Authors:**

- Manuel Beschi (<mailto:manuel.beschi@unibs.it>)
- Nicola Pedrocchi (<mailto:nicola.pedrocchi@stiima.cnr.it>)  

_Software License Agreement (BSD License)_
_Copyright (c) 2017, National Research Council of Italy, Institute of Industrial Technologies and Automation_
_All rights reserved._
