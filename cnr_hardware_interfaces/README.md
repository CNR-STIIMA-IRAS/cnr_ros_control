# Hardware Interfaces

The metapackage is an extension of the [hardware_interface](https://github.com/ros-controls/ros_control/tree/noetic-devel/hardware_interface).

Specifically, the extension built up on five packages:

1. `cnr_hardware_interface` [(readme)](cnr_hardware_interface/cnr_hardware_interface/README.md): this package is **fully compatible with [ros_control](http://wiki.ros.org/ros_control "ros_control")**. It implements a derived class  `cnr_hardware_interface::RobotHW` that inherits the standard `RobotHW`. The class implements some useful functionalities: 
    - It Integrates a [diagnostics](http://wiki.ros.org/diagnostics) module, to track the times of the read/write and update functions 
    - It provides a tracking of the state of the class, and it reacts in case of error
    - It provides a `prepareSwitch` function to ease the inherited classes
    - It integrares a [logger module](www.www) that tracks the input and ouptus of the function, and generates log file for the class.
2. `cnr_hardware_nodelet_interface` [(readme)](cnr_hardware_interfaces/cnr_hardware_nodelet_interface/README.md): this package is a set of utilities that ease the lifecycle management of the `RobotHW` in a `nodelet`. The package allows loading and downloading of a nodelet, and inside the nodelet a infinite-loop is packed in a thread. The loop implements the read/update/write architecture of the **ros_control** architecture. The `RobotHW` is uploaded using the [pluginlib](http://wiki.ros.org/pluginlib), and the actual robot can be easily uploaded just thoguh a parameter, that corresponds to the type of the RobotHW implementation (tha should be properly configured. See [here](cnr_hardware_interfaces/cnr_hardware_interface/README.md)
3. `cnr_topic_hardware_interface` [(readme)](cnr_hardware_interfaces/cnr_topic_hardware_interface/README.md): this pakcage is a specialization of the abstract `RobotHW` class in `cnr_hardware_interface`. Specifically: 
   - It subscribes/publishes a topic of type `joint_states` and it translates the topic field inside the proper handle structures. It allows a generic controller to access the topic field as it is a standard `hardware_interface`
   - It exports the class using the pluginlib, in order to ease the dynamic loading of the class. 
4. `cnr_topics_hardware_interface` [(readme)](cnr_hardware_interfaces/cnr_topics_hardware_interface/README.md): this package is similar to the previous, but it allows the publication and subscription of a set of topics, and it manages more topic types. 
5. `cnr_fake_hardware_interface` [(readme)](cnr_hardware_interfaces/cnr_fake_hardware_interface/README.md): This packae implements a fake `RobotHW` that receives the joint position commands, and it copies the information on the feedback fields. It is useful for debugging and simulation.