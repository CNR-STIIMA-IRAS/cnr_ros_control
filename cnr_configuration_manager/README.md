# Configuration (aka metacontroller) manager developed by CNR-ITIA (www.itia.cnr.it)

The repository contains the implementation of a controller manager which load and unload multiple hardware interfaces and the related controllers.
The package is developed by the Institute of Industrial Technologies and Automation, of the National Research Council of Italy (CNR-ITIA).


# Functionalities and code organization

The package **cnr_configuration_manager** is made the following classes: 

1) **ConfigurationManager** in the core of the package. It provides the services (defined in **configuration_msgs** package) to start/stop/list configuration.

The following node are provided:

1) **cnr_configuration_manager** instantiates the **ConfigurationManager** class in a node.

## Service availables


- "/configuration_manager/start_configuration" [type: configuration_msgs::StartConfiguration] start configuration:

> _start_configuration_: name of the desired configuration. 

> _strictness_ level can be equal to:

> > 1: same behavior of ros control package (if the client ask to run an already running controller it will be leaved running)

> > 2: same behavior of ros control package

> > 0: like 1 but if the client ask to run an already running controller it will be restarted as well as its hardware interface

- "/configuration_manager/stop_configuration" [type: configuration_msgs::StopConfiguration] stop running configuration:

> _strictness_ level can be equal to:

> > 1: same behavior of ros control package (if the client ask to run an already running controller it will be leaved running)

> > 2: same behavior of ros control package

> > 0: like 1 but if the client ask to run an already running controller it will be restarted as well as its hardware interface

- "/configuration_manager/list_configurations" [type: configuration_msgs::ListControllers] provides the list of available configurations and their status (running / loaded)


## Configurations

Configurations (aka metacontrollers) are a set of controllers running on a set of hardware interface.

A common issue of ROS control architecture is the absence of toolchain functionality. For instance, a cascade control architecture has to be implemented in a single controller since primary controller cannot consider the secondary control as its own hardware interface.
Moreover, it manages in a effective way the controllers lifecycle but it does not manage the hardware interface lifecycle.

To overcome these issues, itia hardware interfaces are developed as nodelets (see cnr_hardware_interface packages for details) which can be dynamically loaded/unloaded. Moreover, cnr_topic_hardware_interface can be used in higher layers of complex control architectures. When it publishes/subscribes topics from the other hardware interface (or the controllers loaded on them) the messages are shared by means of nodelet fashion (shared memory instead of tcp communications) making the communications faster and more reliable.
ROS bonds ensure that the system will be stopped if one hardware interface fails.

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
        controller        : "controller_3" #it is strongly recommended (but not mandatory) to use different names also for controllers loaded in different hardware interfaces
  - name: "configuration2"
    components: 
      - description: "a brief description of the component"
        hardware_interface: "motor_velocity_hi"
        controller        : "linkpos_to_vel_jnt1_1"
      - description: "a brief description of the component" #different configurations can use the same hardware interfaces or the same controllers
        hardware_interface: "hardware_interface_1"
        controller        : "controller_2" 
```

## Example of use
Load a configuration:
```
rosservice call /configuration_manager/start_configuration "start_configuration: 'configuration1'
strictness: 1"
```

Force reload a configuration:
```
rosservice call /configuration_manager/start_configuration "start_configuration: 'configuration1'
strictness: 0"
```

Stop a configuration:
```
rosservice call /configuration_manager/stop_configuration "strictness: 0"
```

List configuration
```
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