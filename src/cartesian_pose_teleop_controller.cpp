// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_irom_controllers/cartesian_pose_teleop_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_irom_controllers {

bool CartesianPoseTeleopController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseTeleopController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseTeleopController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseTeleopController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseTeleopController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseTeleopController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseTeleopController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianPoseTeleopController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
  lastCommand_time_ = ros::Duration(0.0);
  start_pose_ = initial_pose_;
  x_offset = 0;
  lastPrint_time_ = ros::Duration(0.0);

  keep = true;
  count = 0;

  x_command_sub_ = n.subscribe("/spacenav/offset", 100, &CartesianPoseTeleopController::storeCommand, this);
}

void CartesianPoseTeleopController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  
  if(keep)
  {
    x_command_sub_ = n.subscribe("/spacenav/offset", 100, &CartesianPoseTeleopController::storeCommand, this);

    cartesian_pose_handle_->setCommand(current_pose_);
    count++;
    if(count > 100)
    {
      lastCommand_time_ = elapsed_time_;
      count = 0;
      keep = false;
      x_command_sub_.shutdown();
    }
    return;
  }

  if((elapsed_time_.toSec()- lastCommand_time_.toSec()) < 1.0)
  {
    std::array<double, 16> new_pose_ = start_pose_;
    new_pose_[14] -= x_offset*(elapsed_time_.toSec()-lastCommand_time_.toSec())/1000.0;

  // new_pose_[14] -= 0.000002;
    if((elapsed_time_.toSec() - lastPrint_time_.toSec()) > 0.0001)
    {
      ROS_INFO("Current x: %f, command: %f\n", current_pose_[14], new_pose_[14]);
      lastPrint_time_ = elapsed_time_;
    }
    cartesian_pose_handle_->setCommand(new_pose_);
  }
  else
  {
    start_pose_ = current_pose_;
    x_offset = 10*x_command;  // update every 0.1s

    cartesian_pose_handle_->setCommand(current_pose_);
    keep = true;
  }
}

void CartesianPoseTeleopController::storeCommand(const geometry_msgs::Vector3::ConstPtr& msg)
{
  x_command = msg->x;
  // for (size_t i = 0; i < 7; ++i) {
  //   command_pose_[i] = msg->data[i];
  // }  
}


}  // namespace franka_irom_controllers

PLUGINLIB_EXPORT_CLASS(franka_irom_controllers::CartesianPoseTeleopController,
                       controller_interface::ControllerBase)
