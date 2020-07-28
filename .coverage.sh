#!/bin/bash

source "${ICI_SRC_PATH}/workspace.sh"
source "${ICI_SRC_PATH}/util.sh"

echo "Generating coverage for cnr_logger"

ws=~/target_ws
extend="/opt/ros/$ROS_DISTRO"

pacakges_to_coverage=("cnr_topics_hardware_interface"\
                      "cnr_topic_hardware_interface"\
                      "cnr_hardware_nodelet_interface"\
                      "cnr_hardware_interface"\
                      "cnr_controller_manager_interface"\
                      "cnr_controller_interface"\
                      "cnr_configuration_manager")

# Print array values in  lines
for package in ${pacakges_to_coverage[*]}; do
     echo "$package"
     ici_exec_in_workspace "$extend" "$ws" catkin build "$package" -v --no-deps --catkin-make-args "$package_coverage_report"

     echo "Uploading coverage results for $package to codecov.io"

     # Remove duplicated information
     rm "$ws/build/cnr_logger/$package _coverage_report.info.cleaned"
     rm "$ws/build/cnr_logger/$package _coverage_report.info.removed"

     # Actually upload coverage information
     bash <(curl -s https://codecov.io/bash) -s "$ws/build/$package/"

done

