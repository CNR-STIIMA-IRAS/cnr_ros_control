/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_hardware_interface/cnr_robot_hw.h>
#include <gtest/gtest.h>

std::shared_ptr<cnr_hardware_interface::RobotHW> robot;
std::shared_ptr<cnr_logger::TraceLogger>           logger;

// Declare a test
TEST(TestSuite, robotHwFullConstructor)
{

  EXPECT_NO_FATAL_FAILURE(logger.reset(new cnr_logger::TraceLogger("log1", "/file_and_screen_different_appenders")));
  EXPECT_FALSE(logger->init("/file_and_screen_different_appenders", false, false));  // Already initialized

  EXPECT_NO_FATAL_FAILURE(robot.reset(new cnr_hardware_interface::RobotHW( )));

  EXPECT_NO_FATAL_FAILURE(robot.reset());
  EXPECT_NO_FATAL_FAILURE(logger.reset());
}

// Init
TEST(TestSuite, robotHwFullInit)
{

  EXPECT_NO_FATAL_FAILURE(logger.reset(new cnr_logger::TraceLogger("log1", "/file_and_screen_different_appenders")));
  EXPECT_FALSE(logger->init("/file_and_screen_different_appenders", false, false));  // Already initialized

  EXPECT_NO_FATAL_FAILURE(robot.reset(new cnr_hardware_interface::RobotHW( )));
  ros::NodeHandle robot_nh("/robot_hw");
  ros::NodeHandle root_nh ("/");
  ros::NodeHandle does_not_exist_nh ("/");
  EXPECT_TRUE(  robot->init(root_nh, robot_nh));
  EXPECT_FALSE( robot->init(root_nh, does_not_exist_nh));  // default vaules depiste none parameters
  EXPECT_FALSE( robot->init(does_not_exist_nh, robot_nh));  // default vaules depiste none parameters

  EXPECT_NO_FATAL_FAILURE(robot.reset());
  EXPECT_NO_FATAL_FAILURE(logger.reset());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "cnr_logger_tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
