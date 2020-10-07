# cnr_controller_interfaces

The metapackage is built up on two packages:

1. `cnr_controller_interface` [(readme)](cnr_controller_interfaces/cnr_controller_interface/README.md): 
   this package is **fully compatible with [ros_control](http://wiki.ros.org/ros_control "ros_control")**.
   It implements a  template derived by  `::controller_interface::Controller< T >` 
   [(link)](https://github.com/ros-controls/ros_control/blob/noetic-devel/controller_interface/include/controller_interface/controller.h). 
   The class implements some useful functionalities:
   1. It Integrates a [diagnostics](http://wiki.ros.org/diagnostics) module, to track the times of the read/write and update functions
   2. It provides a tracking of the state of the class, and it reacts in case of error
   3. It provides a `ros::CallbackQueue` to control the flow of the topics managed by the controller
   4. It integrares a [logger module](www.www) that tracks the input and ouptus of the function, and generates log file for the class.

2. `cnr_controller_manager_interface` [(readme)](cnr_controller_interfaces/cnr_controller_manager_interface/README.md): this package is **fully compatible with [ros_control](http://wiki.ros.org/ros_control "ros_control")**. The package is a set of utilities to load/unload start and stop the controller. The controllers namespace are constrained to couple the controllers parameters with the proper hardware_interface parameter.
