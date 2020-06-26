// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>

namespace franka_irom_controllers {

class CartesianPoseTeleopController: 
      public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;

  ros::Duration elapsed_time_;
  ros::Duration lastCommand_time_;
  ros::Duration lastPrint_time_;

  std::array<double, 16> initial_pose_{};
  std::array<double, 16> current_pose_{};
  std::array<double, 16> start_pose_{};
  std::array<double, 16> end_pose_{};

  ros::Subscriber x_command_sub_;
  double x_command;
  double x_offset;
  bool keep;
  double count;
  
  // std::array<double, 7> command_pose_{};
  // std::array<double, 7> current_pose_{};

  ros::NodeHandle n;
  void storeCommand(const geometry_msgs::Vector3::ConstPtr& msg);
};

}  // namespace franka_irom_controllers
