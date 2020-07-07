# Topics hardware interface

The packages implements an hardware interface which reads from a JointState topic and write in another JointState topic.
It can be used in the configurations managed by the cnr_controller_manager.

The repository contains the implementation of a hardware interface derived from **cnr_hardware_interface**.
The package is developed by the Institute of Industrial Technologies and Automation, of the National Research Council of Italy (CNR-ITIA).

## Functionalities and code organization

The package **cnr_topics_hardware_interface** is made the class **TopicsRobotHW**, which implements some additional features wrt to **BasicRobotHW**:

> _checkForConflict_ policy is different from ROS-Control standard.
> Each controller can use more than a hardware_interface for a single joint (for example: position, velocity, effort).
> One controller can control more than one joint.
> A joint can be used only by a controller.

## Parameters

Topics hardware interface requires the following parameters:

```yaml
hardware_interface_name:
  type: "itia/control/TopicHwIfaceNodelet"
  joint_names: 
  - "joint_1"
  - "joint_2"
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

- Manuel Beschi (<mailto:manuel.beschi@stiima.cnr.it>)
- Nicola Pedrocchi (<mailto:nicola.pedrocchi@stiima.cnr.it>)  

_Software License Agreement (BSD License)_
_Copyright (c) 2017, National Research Council of Italy, Institute of Industrial Technologies and Automation_
_All rights reserved._
