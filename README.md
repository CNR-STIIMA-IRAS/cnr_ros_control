# Extension of ROS-CONTROL #

[![CI][a]][1]
[![codecov][c]][3]
[![Codacy Badge][d]][4]

## The design of the framework ##

The repository contains the implementation of an exstension of the [ros_control](http://wiki.ros.org/ros_control "ros_control") framework
developed by the Institute of Intelligent Industrial Technologies and Systems for Advanced Manufacturing [(STIIMA)](http://www.stiima.cnr.it),
of the National Research Council of Italy [(CNR)](www.cnr.it).

The extension is based on one package and two meta-packages.

* The `cnr_hardware_interfaces` [(readme)](cnr_hardware_interfaces/README.md)
* The `cnr_controller_interfaces` [(readme)](cnr_hardware_interfaces/README.md)
* The `cnr_configuration_manager` [(readme)](cnr_configuration_manager/README.md)

The core of the extension is having encapsulated the lifecyle of the `RobotHw` [(link)](https://github.com/ros-controls/ros_control/wiki/hardware_interface) in a [`nodelet`](http://wiki.ros.org/nodelet).

This design choice allows the implementation of cascade controller architectures, preserving the fastest communication as possible between the nodelets consituting the controller architecture.

A special `RobotHw` called `TopicRobotHw` is provided by the `cnr_hardware_interfaces` package, and it has been designed in order to encaspulate generic topics in the standard `hardware_interface` [(link)](https://github.com/ros-controls/ros_control/wiki/hardware_interface), and therefore to allows the controller accessing the topic content using the handle-based mechanism.  

[![Alternate Text](.media/Picture1.png)](https://youtu.be/JGmhB_9PjZk "Online presentation")

## Example of Usage ##

Each of the packages has his own README (follow the hyperlink for further information on each package)

The core of the extension is the `cnr_configuration_manager` [(readme)](cnr_configuration_manager/README.md).

The node has been desgined to allow a dynamic configuration of the controller architecture.

First, it allows the dynamic loading/unloading of many `RobotHW` in parallel encapsulating each `RobotHW` in a different nodelet.
Then, it allows the loading/unlaoding and start/stop of all the different controllers needed.

The configuration is done trough a proper yaml file, and it is possible to dynamically switch between many different configurations.

A collection of Matlab scripts useful for the...

## Dependencies ##

The package provides a .rosinstall file with the dependencies, and the dependencies are configured in the `package.xml` of the packages.

```shell
$ export ROSINSTALL_FILE=dependencies.rosinstall
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/CNR-STIIMA-IRAS/cnr_ros_control
$ wstool init
$ wstool merge cnr_ros_control/dependencies.rosinstall
$ wstool up -v
$ cd ~/catkin_ws
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

## Example to download ##

TODO

## Developer Contact ##

**Authors:**

* Manuel Beschi (<mailto::manuel.beschi@stiima.cnr.it>)
* Nicola Pedrocchi (<mailto::nicola.pedrocchi@stiima.cnr.it>)
* Stefano Ghidini (<mailto::stefano.ghidini@stiima.cnr.it>)

_Software License Agreement (BSD License)_
_Copyright (c) 2020, National Research Council of Italy, Institute of Intelligent Industrial Technologies and Systems for Advanced Manufacturing_
_All rights reserved._

## Acknowledge ##

![EC-H2020](.media/Sharework_logo_color_250px.png) [ShareWork webpage](https://sharework-project.eu/)

![EC-H2020](.media/flag_yellow_low-300x201.jpg)

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 820807.
This website reflects only the author’s view and the European Commission is not responsible for any use that may be made of the information it contains.|

[a]:https://github.com/CNR-STIIMA-IRAS/cnr_ros_control/actions/workflows/industrial_ci_action.yml/badge.svg
[1]:https://github.com/CNR-STIIMA-IRAS/cnr_ros_control/actions/workflows/industrial_ci_action.yml

[b]:https://travis-ci.com/CNR-STIIMA-IRAS/cnr_ros_control.svg?branch=master
[2]:https://travis-ci.com/CNR-STIIMA-IRAS/cnr_ros_control

[c]:https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_ros_control/branch/master/graph/badge.svg
[3]:https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_ros_control

[d]:https://api.codacy.com/project/badge/Grade/fd683d54c39443df9c685fb189300695
[4]:https://app.codacy.com/gh/CNR-STIIMA-IRAS/cnr_ros_control?utm_source=github.com&utm_medium=referral&utm_content=CNR-STIIMA-IRAS/cnr_ros_control&utm_campaign=Badge_Grade_Dashboard