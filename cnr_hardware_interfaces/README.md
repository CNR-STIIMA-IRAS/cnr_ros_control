# Hardware Interfaces

The metapackage is an extension of the [hardware_interface](https://github.com/ros-controls/ros_control/tree/noetic-devel/hardware_interface).

Specifically, the extension built up on five packages:

## CNR Hardware Interface [(readme)](cnr_hardware_interface/README.md)

The `cnr_hardware_interface` [(readme)](cnr_hardware_interface/cnr_hardware_interface/README.md) is **fully compatible with [ros_control](http://wiki.ros.org/ros_control "ros_control")**. It implements a derived class  `cnr_hardware_interface::RobotHW` that inherits the standard `RobotHW`. The class implements some useful functionalities:

1. It Integrates a [diagnostics](http://wiki.ros.org/diagnostics) module, to track the times of the read/write and update functions
2. It provides a tracking of the state of the class, and it reacts in case of error
3. It provides a `prepareSwitch` function to ease the inherited classes
4. It integrares a [logger module](www.www) that tracks the input and ouptus of the function, and generates log file for the class.

## CNR Hardware Nodelet Interface [(readme)](cnr_hardware_nodelet_interface/README.md)

The  `cnr_hardware_nodelet_interface` [(readme)](cnr_hardware_nodelet_interface/README.md) is the core of the **ros_control** extension

> this package is a set of utilities that manages the lifecycle of the `RobotHW` in `nodelet`.

The package provides classes that allows the loading and downloading of a `nodelet`, and inside the `nodelet` a infinite-loop `read()`-`update()`-`write()` is packed in a thread.

The `RobotHW` is uploaded using the [pluginlib](http://wiki.ros.org/pluginlib), and the actual robot can be easily uploaded just thoguh a parameter, that corresponds to the type of the RobotHW implementation (tha should be properly configured. See [here](cnr_hardware_interface/README.md)

## CNR Topic Hardware Interface [(readme)](cnr_topic_hardware_interface/README.md)

The package `cnr_topic_hardware_interface` [(readme)](cnr_topic_hardware_interface/README.md)  a specialization of the abstract `RobotHW` class in `cnr_hardware_interface`. 

> The `TopicRobotHW` subscribes and/or publishes a topic of type `joint_states` and it translates the topic field inside the proper handle structures. It allows a generic controller to access the topic field as it is a standard `hardware_interface`

The implementation follows the standard `RobotHW`, and, it only exports the class using the `pluginlib`, in order to ease the dynamic loading of the class.

## CNR Topics Hardware Interface [(readme)](cnr_topic_hardware_interface/README.md)

The package `cnr_topics_hardware_interface` [(readme)](cnr_topics_hardware_interface/README.md) is similar to the previous, but it allows the publication and subscription of a set of topics, and it manages more topic types.

## CNR Fake Hardware Interface [(readme)](cnr_topic_hardware_interface/README.md)

The package `cnr_fake_hardware_interface` [(readme)](cnr_fake_hardware_interface/README.md) implements a fake `RobotHW` that receives the joint position commands, and it copies the information on the feedback fields. It is useful for debugging and simulation.
