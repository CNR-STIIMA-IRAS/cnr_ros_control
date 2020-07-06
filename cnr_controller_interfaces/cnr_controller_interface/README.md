# nr_controller_interface#

Package to publish JointStates in nodelet fashion (shared memory). It allows sorting the joint_names.
The published topic has this name:

_/your_robot_hardware/your_controller_name/joint_states_

use _nodelet_ remapping parameters to change it.

Plugins list:

- *ros/control/JointStatePublisher*




### parameters ###
```yaml
  type: "ros/control/JointStatePublisher"

  your_controller_name:
    type: "ros/control/JointStatePublisher"
    controlled_joint: ['joint_1','joint_2',....] # if not specified, use all joints
```