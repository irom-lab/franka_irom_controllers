// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_irom_controllers/cartesian_velocity_teleop_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_irom_controllers {

bool CartesianVelocityTeleopController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityTeleopController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityTeleopController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityTeleopController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityTeleopController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianVelocityTeleopController: Robot is not in the expected starting position "
            "for running this example. Run `roslaunch franka_example_controllers "
            "move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
            "first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityTeleopController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianVelocityTeleopController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  lastCommand_time_ = ros::Duration(0.0);
  lastPrint_time_ = ros::Duration(0.0);

  initial_pose_ = velocity_cartesian_handle_->getRobotState().O_T_EE_d;
  target_z = 0.15;

  z_offset = 0;
  prev_vel_z = 0.0;
  speed = 0.1;

  x_command_sub_ = n.subscribe("/spacenav/offset", 100, &CartesianVelocityTeleopController::storeCommand, this);
}

void CartesianVelocityTeleopController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  elapsed_time_ += period;
  current_pose_ = velocity_cartesian_handle_->getRobotState().O_T_EE_d;
  std::array<double, 6> new_vel_ = {{0, 0, 0, 0.0, 0.0, 0.0}};
  
  if((elapsed_time_.toSec()- lastCommand_time_.toSec()) < 1.0)
  {
    double vel_z = target_z - current_pose_[14];
    ROS_INFO("vel_z: ", vel_z);
    if (vel_z < 0.02) {
        vel_z = 0.9*prev_vel_z;
    }
    else {
      vel_z = speed*vel_z;
    }
    vel_z = 0.99*prev_vel_z + 0.01*vel_z;
    prev_vel_z = vel_z;

    new_vel_[2] = vel_z;

    // new_vel_[0] -= x_offset*(elapsed_time_.toSec()-lastCommand_time_.toSec())/1000.0;

    if((elapsed_time_.toSec() - lastPrint_time_.toSec()) > 0.0001)
    {
      ROS_INFO("Command: %f\n", new_vel_[2]);
      lastPrint_time_ = elapsed_time_;
    }
  }
  else
  {
    target_z = current_pose_[14] + z_offset;
    new_vel_[2] = prev_vel_z;
  }
  
  velocity_cartesian_handle_->setCommand(new_vel_);

  // double time_max = 4.0;
  // double v_max = 0.05;
  // double angle = M_PI / 4.0;
  // double cycle = std::floor(
  //     pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max)) / time_max));
  // double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * elapsed_time_.toSec()));
  // double v_x = std::cos(angle) * v;
  // double v_z = -std::sin(angle) * v;
  // std::array<double, 6> command = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
  // ROS_INFO("X: %f, Z:%f", v_x, v_z);
  // velocity_cartesian_handle_->setCommand(command);
}

void CartesianVelocityTeleopController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void CartesianVelocityTeleopController::storeCommand(const geometry_msgs::Vector3::ConstPtr& msg)
{
  z_offset = 0.1*msg->x;
}

}  // namespace franka_irom_controllers

PLUGINLIB_EXPORT_CLASS(franka_irom_controllers::CartesianVelocityTeleopController,
                       controller_interface::ControllerBase)
