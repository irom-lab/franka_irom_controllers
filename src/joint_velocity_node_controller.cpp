// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_irom_controllers/joint_velocity_node_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>



namespace franka_irom_controllers {

bool JointVelocityNodeController::init(
						hardware_interface::RobotHW* robot_hardware,
                        ros::NodeHandle& node_handle) {

  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR("JointVelocityNodeController: Error getting velocity joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityNodeController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityNodeController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityNodeController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocityNodeController: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle("panda_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointVelocityNodeController: Exception getting state handle: " << e.what());
    return false;
  }

  node_handle.param<double>("max_duration_between_commands", max_duration_between_commands, 0.01);

  // // Rate Limiting
  // if(!node_handle.getParam("rate_limiting/kMaxJointVelocity", max_velocity)) {
  //   ROS_ERROR("JointVelocityNodeController: Could not get parameter rate_limiting/kMaxJointVelocity");
  //   return false;
  // }
  // if(!node_handle.getParam("rate_limiting/kMaxJointAcceleration", max_acceleration)) {
  //   ROS_ERROR("JointVelocityNodeController: Could not get parameter rate_limiting/kMaxJointAcceleration");
  //   return false;
  // }
  // if(!node_handle.getParam("rate_limiting/kMaxJointJerk", max_jerk)) {
  //   ROS_ERROR("JointVelocityNodeController: Could not get parameter rate_limiting/kMaxJointJerk");
  //   return false;
  // }

  // Stop at collision 
  node_handle.param<bool>("stop_on_contact", stop_on_contact, true);

  // Subscribe to cmd topic
  velocity_command_subscriber = node_handle.subscribe("joint_velocity",
                                                       10,
                &JointVelocityNodeController::joint_velocity_callback,
                                                       this);

//   try {
//     auto state_handle = state_interface->getHandle("panda_robot");

//     std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
//     for (size_t i = 0; i < q_start.size(); i++) {
//       if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
//         ROS_ERROR_STREAM(
//             "JointVelocityNodeController: Robot is not in the expected starting position for "
//             "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
//             "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
//         return false;
//       }
//     }
//   } catch (const hardware_interface::HardwareInterfaceException& e) {
//     ROS_ERROR_STREAM(
//         "JointVelocityNodeController: Exception getting state handle: " << e.what());
//     return false;
//   }

  return true;
}

void JointVelocityNodeController::starting(const ros::Time& /* time */) {
  velocity_command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};  // 7 joints
  last_sent_velocity = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
}


void JointVelocityNodeController::joint_velocity_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
// void JointVelocityNodeController::joint_velocity_callback(const std::array<double, 7>& msg) {
  // Callback for ROS message
  for (size_t i = 0; i < 7; ++i) {
		velocity_command[i] = msg->data[i];
	}
  // velocity_command = msg;
  time_since_last_command = ros::Duration(0.0);
}

void JointVelocityNodeController::update(const ros::Time& /* time */,
                                        const ros::Duration& period) {
  // Update the controller at 1kHz
  time_since_last_command += period;

  // If no message received in set time,
  if(time_since_last_command.toSec() > max_duration_between_commands) {
    velocity_command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  }

  auto state = state_handle_->getRobotState();

  // Check for contacts
  if(stop_on_contact) {
    for (size_t i = 0; i < state.cartesian_contact.size(); i++) {
      if(state.cartesian_contact[i]) {
        velocity_command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        // ROS_ERROR_STREAM("Detected Cartesian Contact in Direction "  << i);
      }
    }
  }

  // Use rate limiter
  // last_sent_velocity = franka::limitRate(
  //   max_velocity,
  //   max_acceleration,
  //   max_jerk,
  //   velocity_command,
  //   state.O_dP_EE_c,  // TODO
  //   state.O_ddP_EE_c
  // );

  // Send joint velocity cmd to arm
  for (size_t i = 0; i < 7; ++i) {
    velocity_joint_handles_[i].setCommand(velocity_command[i]);
  }
  // velocity_joint_handles_->setCommand(velocity_command);

//   ros::Duration time_max(8.0);
//   double omega_max = 0.1;
//   double cycle = std::floor(
//       std::pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max.toSec())) /
//                          time_max.toSec()));
//   double omega = cycle * omega_max / 2.0 *
//                  (1.0 - std::cos(2.0 * M_PI / time_max.toSec() * elapsed_time_.toSec()));

//   for (auto joint_handle : velocity_joint_handles_) {
//     joint_handle.setCommand(omega);
//   }
}

void JointVelocityNodeController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_irom_controllers

PLUGINLIB_EXPORT_CLASS(franka_irom_controllers::JointVelocityNodeController,
                       controller_interface::ControllerBase)
