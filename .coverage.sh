#!/bin/bash

declare -a StringArray=("cnr_configuration_manager" "cnr_controller_interface" "cnr_controller_interface_params"\
                            "cnr_hardware_driver_interface" "cnr_fake_hardware_interface" "cnr_topic_hardware_interface"\
                                "cnr_topics_hardware_interface" )

ws=~/target_ws

cp /home/runner/work/cnr_ros_control/cnr_ros_control/codecov.yml "$ws"/

cd "$ws"

for val in ${StringArray[@]}; do

    echo "Generating coverage for '$val'"

    catkin build "$val" -v --no-deps --catkin-make-args run_tests
    catkin build "$val" --no-deps --catkin-make-args coverage_report --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -DCATKIN_ENABLE_TESTING=ON

done

# Remove duplicated information
find "$ws" -name \*.info.cleaned -exec rm {} \;
find "$ws" -name \*.info.removed -exec rm {} \;
