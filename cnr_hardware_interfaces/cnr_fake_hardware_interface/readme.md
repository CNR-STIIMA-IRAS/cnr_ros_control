```yaml
ur5_hw:  # name of the hardware interface
  type           : cnr/control/FakeRobotHW  # type
  appenders      : [file, screen]  # logging information: the first one is stored in a file, the second one in the screen
  levels         : [trace, debug]  # trace debug info warn error fatal
  pattern_layout : "[%5p][%d{HH:mm:ss,SSS}][%50M:%04L][%24c] %m%n"
  file_name      : ur5_hw  # name of the file (it is saved in ~/.ros/
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


  initial_position: [2.5139003020721433, -1.300169384679926, 1.2500994437535364, -1.5412232868185847, -1.5994267077911786, 5.39879464423676]  # optional. Initial position  
```