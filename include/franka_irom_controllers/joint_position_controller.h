// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <std_msgs/Float64MultiArray.h>

namespace franka_example_controllers {

class JointPositionController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  // std::array<double, 7> initial_pose_{};

  ros::Subscriber jog_joint_command_sub_;
  std::array<double, 7> command_pose_{};
  std::array<double, 7> current_pose_{};

  ros::NodeHandle n;
  void storeJoint(const std_msgs::Float64MultiArray::ConstPtr& msg);
};

}  // namespace franka_example_controllers
