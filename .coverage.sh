#!/bin/bash

source "${ICI_SRC_PATH}/workspace.sh"
source "${ICI_SRC_PATH}/util.sh"

declare -a StringArray=("cnr_configuration_manager" "cnr_controller_interface" "cnr_controller_interface_params"\
                            "cnr_hardware_driver_interface" "cnr_fake_hardware_interface" "cnr_topic_hardware_interface" 
                                "cnr_topics_hardware_interface" "cnr_ros_control_test_description")

for val in ${StringArray[@]}; do
   echo 

    echo "Generating coverage for '$val'"

    ws=~/target_ws
    extend="/opt/ros/$ROS_DISTRO"
    ici_exec_in_workspace "$extend" "$ws" catkin build $val -v --no-deps --catkin-make-args coverage_report

    echo "Uploading coverage results to codecov.io"

    # Remove duplicated information
    rm "$ws/build/$val/coverage_report.info.cleaned"
    rm "$ws/build/$val/coverage_report.info.removed"

    # Actually upload coverage information
    bash <(curl -s https://codecov.io/bash) -s "$ws/build/$val/"

done