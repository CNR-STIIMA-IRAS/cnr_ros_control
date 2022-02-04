# Hardware Interfaces

The metapackage is an extension of the [hardware_interface](https://github.com/ros-controls/ros_control/tree/noetic-devel/hardware_interface).

Specifically, the extension built on five packages:

1. `cnr_hardware_nodelet_interface`
2. `cnr_topic_hardware_interface`
3. `cnr_topics_hardware_interface`
4. `cnr_fake_hardware_interface`

They requires the package [`cnr_hardware_interface`](https://github.com/CNR-STIIMA-IRAS/cnr_hardware_interface). 

## CNR Hardware Nodelet Interface [(readme)](cnr_hardware_nodelet_interface/README.md)

The  `cnr_hardware_nodelet_interface` [(readme)](cnr_hardware_nodelet_interface/README.md) is the core of the **ros_control** extension

> this package is a set of utilities that manages the lifecycle of the `RobotHW` in `nodelet`.

The package provides classes that allows the loading and downloading of a `nodelet`, and inside the `nodelet` a infinite-loop `read()`-`update()`-`write()` is packed in a thread.

The `RobotHW` is uploaded using the [pluginlib](http://wiki.ros.org/pluginlib), and the actual robot can be easily uploaded just thoguh a parameter, that corresponds to the type of the RobotHW implementation (tha should be properly configured. See [here](https://github.com/CNR-STIIMA-IRAS/cnr_hardware_interface/blob/main/README.md)

## CNR Topic Hardware Interface [(readme)](cnr_topic_hardware_interface/README.md)

The package `cnr_topic_hardware_interface` [(readme)](cnr_topic_hardware_interface/README.md)  a specialization of the abstract `RobotHW` class in [`cnr_hardware_interface`](https://github.com/CNR-STIIMA-IRAS/cnr_hardware_interface). 

> The `TopicRobotHW` subscribes and/or publishes a topic of type `joint_states` and it translates the topic field inside the proper handle structures. It allows a generic controller to access the topic field as it is a standard `hardware_interface`

The implementation follows the standard `RobotHW`, and, it only exports the class using the `pluginlib`, in order to ease the dynamic loading of the class.

## CNR Topics Hardware Interface [(readme)](cnr_topic_hardware_interface/README.md)

The package `cnr_topics_hardware_interface` [(readme)](cnr_topics_hardware_interface/README.md) is similar to the previous, but it allows the publication and subscription of a set of topics, and it manages more topic types.

## CNR Fake Hardware Interface [(readme)](cnr_topic_hardware_interface/README.md)

The package `cnr_fake_hardware_interface` [(readme)](cnr_fake_hardware_interface/README.md) implements a fake `RobotHW` that receives the joint position commands, and it copies the information on the feedback fields. It is useful for debugging and simulation.

## Developer Contact

**Authors:**

- Nicola Pedrocchi (<mailto:nicola.pedrocchi@stiima.cnr.it>)  
- Manuel Beschi (<mailto:manuel.beschi@stiima.cnr.it>)

_Software License Agreement (BSD License)_
_Copyright (c) 2017, National Research Council of Italy, Institute of Industrial Technologies and Automation_
_All rights reserved._
