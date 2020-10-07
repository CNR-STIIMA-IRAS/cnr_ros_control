# Configuration (aka metacontroller) manager

The repository contains the implementation of a controller manager which load and unload multiple hardware interfaces
and the related controllers.

The package is developed by the Institute of Intelligent Industrial Technologies and Systems for Advanced Manufacturing,
of the National Research Council of Italy (CNR-ITIA).

## Design

The package **cnr_configuration_manager** provides one node, the `configuration_manager_node` and a few utilities, 
listed below.

The `configuration_manager_node` is designed to dynamically load and unload a set of different `hardware_interface`,
each packed in `nodelet` ([here])(../cnr_hardware_interfaces/README.md), and to load/start/stop/unload a set of
`ros_controllers`.

This set is called `configuration` and the configuration definition has to be uploaded inside the `rosparam` server
though a proper `yaml`.

Then, the `configuration_manager_node` provides a set of services to load and unload properly the configuration

## Configurations

Configurations (aka metacontrollers) are a set of controllers running on a set of hardware interface.

A common issue of ROS control architecture is the absence of toolchain functionality.
For instance, a cascade control architecture has to be implemented in a single controller since primary
controller cannot consider the secondary control as its own hardware interface. Moreover, 
it manages in a effective way the controllers lifecycle but it does not manage the hardware interface lifecycle.

To overcome these issues, the `RobotHW` are packed in `nodelets` 
[see cnr_hardware_interface](../cnr_hardware_interfaces/README.md)  for details) which can be dynamically loaded/unloaded.
Moreover, cnr_topic_hardware_interface can be used in higher layers of complex control architectures.
When it publishes/subscribes topics from the other hardware interface (or the controllers loaded on them)
the messages are shared by means of nodelet fashion (shared memory instead of tcp communications) making 
the communications faster and more reliable.

Configurations are read from the ROS parameter **control_configurations** which has the following structure:

```yaml
control_configurations:
  - name: "configuration1"
    components:
      - description: "a brief description of the component"
        hardware_interface: "hardware_interface_1" # the name of the hardware interface
        controller        : "controller_1"
      - description: "a brief description of the component"
        hardware_interface: "hardware_interface_1"
        controller        : "controller_2"
      - description: "a brief description of the component"
        hardware_interface: "hardware_interface_2"
        controller        : "controller_3" #it is recommended (but not mandatory) to use different 
                                           # names also for controllers loaded in different hardware interfaces
  - name: "configuration2"
    components:
      - description: "a brief description of the component"
        hardware_interface: "motor_velocity_hi"
        controller        : "linkpos_to_vel_jnt1_1"
      - description: "a brief description of the component" # different configurations can use the same hardware
                                                            # interfaces or the same controllers
        hardware_interface: "hardware_interface_1"
        controller        : "controller_2"
```

## Service availables

```shell
/configuration_manager/start_configuration [type: configuration_msgs::StartConfiguration] 
"start_configuration: '' ,
strictness: "
```

where

```shell
start_configuration:  is the label of the configuration to be started. The label is the 'name' field of the
                      control_configurations parameter

strictness: is an integer:
  1: same behavior of `ros_control` if the client asks to run an already running controller it will be leaved running
  2: same behavior of `ros_control`
  0: like 1, but if the client asks to run an already running controller it will be restarted as well as its 
     hardware interface
```

```shell
/configuration_manager/stop_configuration [type: configuration_msgs::StopConfiguration] "stricness: [integer]":
```

where

```shell
strictness: is an integer:
 1: same behavior of `ros_control` if the client assk to run an already running controller it will be leaved running
 2: same behavior of `ros_control` 
 0: like 1, but if the client ask to run an already running controller it will be restarted as well as its hardware interface
```

and finally,

```shell
"/configuration_manager/list_configurations" [type: configuration_msgs::ListControllers] provides the list of available configurations and their status (running / loaded)
```

## Example of use

Load a configuration:

```shell
rosservice call /configuration_manager/start_configuration "start_configuration: 'configuration1'
strictness: 1"
```

Force reload a configuration:

```shell
rosservice call /configuration_manager/start_configuration "start_configuration: 'configuration1'
strictness: 0"
```

Stop a configuration:

```shell
rosservice call /configuration_manager/stop_configuration "strictness: 0"
```

List configuration

```shell
rosservice call /configuration_manager/list_configurations "{}"
```

## Developer Contact

**Authors:**

- Manuel Beschi (<mailto:manuel.beschi@stiima.cnr.it>)
- Nicola Pedrocchi (<mailto:nicola.pedrocchi@stiima.cnr.it>)  
- Stefano Ghidini (<mailto:stefano.ghidini@stiima.cnr.it>)  

_Software License Agreement (BSD License)_
_Copyright (c) 2017, National Research Council of Italy, Institute of Industrial Technologies and Automation_
_All rights reserved._
