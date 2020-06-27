// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <array>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include "ros/ros.h"

#include <franka/rate_limiting.h>
#include <std_msgs/Float64MultiArray.h>


namespace franka_irom_controllers {

class JointVelocityNodeController : public controller_interface::MultiInterfaceController<
									hardware_interface::VelocityJointInterface,
									franka_hw::FrankaStateInterface> {
	public:
		bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
		void update(const ros::Time&, const ros::Duration& period) override;
		void starting(const ros::Time&) override;
		void stopping(const ros::Time&) override;

		// void joint_velocity_callback(const std::array<double, 7>& msg);
		void joint_velocity_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);

	private:
		hardware_interface::VelocityJointInterface* velocity_joint_interface_;
		std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
		std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

		std::array<double, 7> velocity_command;
		std::array<double, 7> last_sent_velocity;
		std::array<double, 7> last_sent_acceleration;
		ros::Duration time_since_last_command;
		ros::Subscriber velocity_command_subscriber;

		// Parameters
		double max_duration_between_commands;
		std::array<double, 7> max_velocity;
		std::array<double, 7> max_acceleration;
		std::array<double, 7> max_jerk;
		bool stop_on_contact;

};

}  // namespace franka_irom_controllers
