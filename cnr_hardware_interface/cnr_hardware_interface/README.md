# hardware_interface from CNR-STIIMA (www.stiima.cnr.it)

This package is **fully compatible with [ros_control](http://wiki.ros.org/ros_control "ros_control")**. 

It implements a derived class  `cnr_hardware_interface::RobotHW` that inherits the standard `RobotHW`. The class implements some useful functionalities:
    - It Integrates a [diagnostics](http://wiki.ros.org/diagnostics) module, to track the times of the read/write and update functions
    - It provides a tracking of the state of the class, and it reacts in case of error
    - It provides a `prepareSwitch` function to ease the inherited classes
    - It integrares a [logger module](www.www) that tracks the input and ouptus of the func