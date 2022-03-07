#!/bin/bash

declare -a StringArray=("cnr_configuration_manager" "cnr_controller_interface" "cnr_controller_interface_params"\
                            "cnr_hardware_driver_interface" "cnr_fake_hardware_interface" "cnr_topic_hardware_interface"\
                                "cnr_topics_hardware_interface" )

ws=~/target_ws
cd "$ws"
catkin config --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -DUSE_ROS=ON -DENABLE_TESTING=ON

for val in ${StringArray[@]}; do

    echo "Generating coverage for '$val'"

    #catkin build "$val" -v --no-deps --catkin-make-args run_tests
    catkin build "$val" -v --no-deps --catkin-make-args coverage_report

    echo "Uploading coverage results to codecov.io"

    # Remove duplicated information
    rm "$ws/build/$val/coverage_report.info.cleaned"
    rm "$ws/build/$val/coverage_report.info.removed"

    # Actually upload coverage information
    bash <(curl -s https://codecov.io/bash) -s "$ws/build/$val/"

done