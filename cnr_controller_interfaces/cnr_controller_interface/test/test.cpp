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


#include <pinocchio/fwd.hpp>


#include <iostream>

#include <ros/node_handle.h>
#include <cnr_logger/cnr_logger.h>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <cnr_fake_hardware_interface/cnr_fake_robot_hw.h>
#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_controller_interface/cnr_joint_controller_interface.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

#include <cnr_controller_interface/cnr_multi_chain_controller_interface.h>

std::shared_ptr<ros::NodeHandle> root_nh;
std::shared_ptr<ros::NodeHandle> robot_nh;
std::shared_ptr<ros::NodeHandle> ctrl_nh;
std::shared_ptr<cnr_hardware_interface::FakeRobotHW> robot_hw;
std::shared_ptr<cnr::control::Controller<hardware_interface::JointStateInterface> > ctrl;


using JointController = cnr::control::JointController<hardware_interface::JointStateHandle,hardware_interface::JointStateInterface>;
using JointCommandController = cnr::control::JointCommandController<hardware_interface::PosVelEffJointHandle,hardware_interface::PosVelEffJointInterface>;

std::shared_ptr<JointController >        jc_ctrl_x;
std::shared_ptr<JointController >        jc_ctrl_6;
std::shared_ptr<JointCommandController > jc_ctrl_cmd_x;
std::shared_ptr<JointCommandController > jc_ctrl_cmd_6;

std::string to_string(const std::vector<std::string>& vv)
{
  std::string ret = "[ ";
  for(const auto &v : vv ) ret += v +" ";
  return ret + "]";
}

//! check the interfaces in the HW
TEST(TestSuite, Handles)
{
  EXPECT_TRUE(robot_hw->get<hardware_interface::JointStateInterface     >());
  EXPECT_TRUE(robot_hw->get<hardware_interface::PositionJointInterface  >());
  EXPECT_TRUE(robot_hw->get<hardware_interface::VelocityJointInterface  >());
  EXPECT_TRUE(robot_hw->get<hardware_interface::EffortJointInterface    >());
  EXPECT_TRUE(robot_hw->get<hardware_interface::PosVelEffJointInterface >());
  EXPECT_TRUE(robot_hw->get<hardware_interface::VelEffJointInterface    >());

  std::cout << "!JointStateInterface     :" << to_string(robot_hw->get<hardware_interface::JointStateInterface     >()->getNames()) << std::endl;
  std::cout << "!PositionJointInterface  :" << to_string(robot_hw->get<hardware_interface::PositionJointInterface  >()->getNames()) << std::endl;
  std::cout << "!VelocityJointInterface  :" << to_string(robot_hw->get<hardware_interface::VelocityJointInterface  >()->getNames()) << std::endl;
  std::cout << "!EffortJointInterface    :" << to_string(robot_hw->get<hardware_interface::EffortJointInterface    >()->getNames()) << std::endl;
  std::cout << "!PosVelEffJointInterface :" << to_string(robot_hw->get<hardware_interface::PosVelEffJointInterface >()->getNames()) << std::endl;
  std::cout << "!VelEffJointInterface    :" << to_string(robot_hw->get<hardware_interface::VelEffJointInterface    >()->getNames()) << std::endl;
}

// Declare a test
TEST(TestSuite, GenericControllerConstructor)
{
  EXPECT_NO_FATAL_FAILURE(ctrl.reset(new cnr::control::Controller<hardware_interface::JointStateInterface>()));
  //EXPECT_FALSE(ctrl->init(robot_hw->get<hardware_interface::JointStateInterface>(), *root_nh, *robot_nh));
  EXPECT_TRUE(ctrl->init(robot_hw->get<hardware_interface::JointStateInterface>(), *robot_nh, *ctrl_nh));
}

TEST(TestSuite, JointControllerXConstructor)
{
  EXPECT_NO_FATAL_FAILURE(jc_ctrl_x.reset(new JointController()));
  //EXPECT_FALSE(jc_ctrl_x->init(robot_hw->get<hardware_interface::JointStateInterface>(), *root_nh, *robot_nh));
  EXPECT_TRUE(jc_ctrl_x->init(robot_hw->get<hardware_interface::JointStateInterface>(), *robot_nh, *ctrl_nh));
}

TEST(TestSuite, JointController6Constructor)
{
  EXPECT_NO_FATAL_FAILURE(jc_ctrl_x.reset(new JointController()));
  //EXPECT_FALSE(jc_ctrl_x->init(robot_hw->get<hardware_interface::JointStateInterface>(), *root_nh, *robot_nh));
  EXPECT_TRUE(jc_ctrl_x->init(robot_hw->get<hardware_interface::JointStateInterface>(), *robot_nh, *ctrl_nh));
}

TEST(TestSuite, JointCommandControllerXConstructor)
{
  EXPECT_NO_FATAL_FAILURE(jc_ctrl_cmd_x.reset(new JointCommandController()));
  //EXPECT_FALSE(jc_ctrl_x->init(robot_hw->get<hardware_interface::PosVelEffJointInterface>(), *root_nh, *robot_nh));
  EXPECT_TRUE(jc_ctrl_cmd_x->init(robot_hw->get<hardware_interface::PosVelEffJointInterface>(), *robot_nh, *ctrl_nh));
}

TEST(TestSuite, JointCommandControllerC6Constructor)
{
  EXPECT_NO_FATAL_FAILURE(jc_ctrl_cmd_6.reset(new JointCommandController()));
  //EXPECT_FALSE(jc_ctrl_6->init(robot_hw->get<hardware_interface::PosVelEffJointInterface>(), *root_nh, *robot_nh));
  EXPECT_TRUE(jc_ctrl_cmd_6->init(robot_hw->get<hardware_interface::PosVelEffJointInterface>(), *robot_nh, *ctrl_nh));
}

TEST(TestSuite, Desctructor)
{
  EXPECT_NO_FATAL_FAILURE(ctrl.reset());
  EXPECT_NO_FATAL_FAILURE(robot_hw.reset());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "cnr_logger_tester");
  root_nh  .reset(new ros::NodeHandle("/"));
  robot_nh .reset(new ros::NodeHandle("/ur10_hw"));
  ctrl_nh  .reset(new ros::NodeHandle("/ur10_hw/fake_controller"));
  robot_hw.reset(new cnr_hardware_interface::FakeRobotHW());
  robot_hw->init(*root_nh, *robot_nh);
  return RUN_ALL_TESTS();
}
