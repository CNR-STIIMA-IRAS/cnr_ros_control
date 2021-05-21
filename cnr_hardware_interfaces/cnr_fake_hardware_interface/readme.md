# Fake Robot HW
This Robot HW set the joint states equal to the joint command.
It has the following hardware interfaces:

- hardware_interface::JointStateInterface     
- hardware_interface::PositionJointInterface  
- hardware_interface::VelocityJointInterface  
- hardware_interface::EffortJointInterface    
- hardware_interface::PosVelEffJointInterface
- hardware_interface::VelEffJointInterface    

The repository contains the implementation of a hardware interface derived from **cnr_hardware_interface**.

The repository is developed by the Institute of Intelligent Industrial Technologies and Systems for Advanced Manufacturing (STIIMA), of the National Research Council of Italy (CNR).

## Configuration
```yaml
ur5_hw:  # name of the hardware interface
  type           : cnr/control/FakeRobotHW  # type
  appenders      : [file, screen]  # logging information: the first one is stored in a file, the second one in the screen
  levels         : [trace, debug]  # trace debug info warn error fatal
  pattern_layout : "[%5p][%d{HH:mm:ss,SSS}][%50M:%04L][%24c] %m%n"
  file_name      : ur5_hw  # name of the file (it is saved in ~/.ros/)
  sampling_period: 0.008
  diagnostic_period: 0.1
  maximum_missing_cycles: 100
  feedback_joint_state_timeout: 10
  joint_names:  # joint names
  - ur5_shoulder_pan_joint
  - ur5_shoulder_lift_joint
  - ur5_elbow_joint
  - ur5_wrist_1_joint
  - ur5_wrist_2_joint
  - ur5_wrist_3_joint
  base_link: ur5_base_link  # root of the chain
  tool_link: ur5_tool0      # endeffector of the chain
  robot_description_param: /robot_description   # URDF descriptor
  robot_description_planning_param: /robot_description_planning  

  # ForceTorqueSensorInterface (force/torque equal to zero)
  wrench_resourse: "wrench"  # optional (defalut: wrench). name of ForceTorqueSensorHandle
  frame_id: "tool0"          # optional (defalut: tool0)
  wrench_topic: "fake_wrench" #optional wrench topic name (default: ~/fake_wrench)


  initial_position: [2.5139003020721433, -1.300169384679926, 1.2500994437535364, -1.5412232868185847, -1.5994267077911786, 5.39879464423676]  # optional. Initial position  
```
