# Topic hardware interface manager is an hardware interface which reads from a JointState topic and write in another JointState topic.
# It can be used in the configurations (aka metacontroller) managed by the cnr_controller_manager. It is developed by CNR-ITIA (www.itia.cnr.it)

The repository contains the implementation of a hardware interface derived from **cnr_hardware_interface**.
The package is developed by the Institute of Industrial Technologies and Automation, of the National Research Council of Italy (CNR-ITIA).


# Functionalities and code organization

The package **cnr_topics_hardware_interface** is made the following classes: 

1) **TopicsRobotHW** implements some additional features wrt to **BasicRobotHW**:
> _checkForConflict_ policy is different from ROS-Control standard. 
  Each controller can use more than a hardware_interface for a single joint (for example: position, velocity, effort).
  One controller can control more than one joint.
  A joint can be used only by a controller.


2) The hardware interface is instantiated on **itia::control::TopicHwIfaceNodelet**, a nodelet that can be (un)loaded by **cnr_controller_manager**. It uses the same functionalities on **itia::control::BasicHwIfaceNodelet**.

## Required parameters

Topic hardware interface requires the following parameters:

```yaml
hardware_interface_name:
  type: "itia/control/TopicHwIfaceNodelet"
  joint_names: 
  - "joint_1"
  - "joint_2"
  allowed_controller_types: 
  - "controller_type1"  
  feedback_joint_state_topic: "/joint_states" # subscribing topic
  command_joint_state_topic: "/joint_command" # publishing topic
  sampling_period: 0.001
  diagnostic_period: 0.1
  feedback_joint_state_timeout: 20 # timeout on subscribing first message.
  # optional
  #remap_source_args: 
  #remap_target_args: 
```

## Developer Contact

**Authors:**   
- Manuel Beschi (manuel.beschi@itia.cnr.it)  
- Nicola Pedrocchi (nicola.pedrocchi@itia.cnr.it)  
 
_Software License Agreement (BSD License)_    
_Copyright (c) 2017, National Research Council of Italy, Institute of Industrial Technologies and Automation_    
_All rights reserved._