import rospy
import actionlib

import moveit_commander
from moveit_commander.conversions import list_to_pose

import franka_gripper.msg
from franka_control.msg import ErrorRecoveryActionGoal
from geometry_msgs.msg import PoseStamped


class PandaCommander(object):
	"""
	PandaCommander is a class which wraps some basic moveit functions for the Panda Robot,
	and some via the panda API
	"""
	def __init__(self, group_name=None):
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()

		self.groups = {}
		self.active_group = None
		self.set_group(group_name)

		# Set workspace, did not work
		# move_group = self.robot.get_group(group_name)
		# move_group.set_workspace([0.0,-0.4,0.005,0.8,0.4,1.0])

		# Limit joint velocities, did not work, set max vel in joint_limits_slow.yaml instead
		# move_group.set_max_velocity_scaling_factor(0.1)
		
		# End effector
		# move_group.set_end_effector_link("panda_link8")
		# self.print_debug_info()


		# Recovery
		self.reset_publisher = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=1)


	def add_table(self):
		table_pose = PoseStamped()
		table_pose.header.frame_id = "panda_link0"
		table_pose.pose.orientation.w = 1.0
		table_pose.pose.position.x = 0.50
		table_pose.pose.position.x = 0.00
		table_pose.pose.position.z = -0.01
		self.scene.add_box("table", table_pose, size=(2.0, 1.0, 0.02))
		self.wait_for_state_update(box_name='table', box_is_known=True, timeout=1.0)


	def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
		start = rospy.get_time()
		seconds = rospy.get_time()
		while (seconds - start < timeout) and not rospy.is_shutdown():
			# Test if the box is in attached objects
			attached_objects = self.scene.get_attached_objects([box_name])
			is_attached = len(attached_objects.keys()) > 0

			# Test if the box is in the scene.
			# Note that attaching the box will remove it from known_objects
			is_known = box_name in self.scene.get_known_object_names()

			# Test if we are in the expected state
			if (box_is_attached == is_attached) and (box_is_known == is_known):
				return True

			# Sleep so that we give other threads time on the processor
			rospy.sleep(0.1)
			seconds = rospy.get_time()

		return False


	def print_debug_info(self):
		if self.active_group:
			planning_frame = self.active_group.get_planning_frame()
			print("============ Reference frame: %s" % planning_frame)
			eef_link = self.active_group.get_end_effector_link()
			print("============ End effector: %s" % eef_link)
		else:
			print("============ No active planning group.")
		print("============ Robot Groups:", self.robot.get_group_names())
		print("============ Printing robot state")
		print(self.robot.get_current_state())
		print("")

	def set_group(self, group_name):
		"""
		Set the active move group
		:param group_name: move group name
		"""
		self.active_group = group_name
		if group_name is None:
			self.active_group = None
			return
		else:
			if group_name not in self.groups:
				if group_name not in self.robot.get_group_names():
					raise ValueError('Group name %s is not valid. Options are %s' % (group_name, self.robot.get_group_names()))
				self.groups[group_name] = moveit_commander.MoveGroupCommander(group_name)
			self.active_group = self.groups[group_name]

	def goto_joints(self, joint_values, group_name=None, wait=True):
		"""
		Move to joint positions.
		:param joint_values:  Array of joint positions
		:param group_name:  Move group (use current if None)
		:param wait:  Wait for completion if True
		:return: Bool success
		"""
		if group_name:
			self.set_group(group_name)
		if not self.active_group:
			raise ValueError('No active Planning Group')

		joint_goal = self.active_group.get_current_joint_values()
		if len(joint_goal) != len(joint_values):
			raise IndexError('Expected %d Joint Values, got %d' % (len(joint_goal), len(joint_values)))
		for i, v in enumerate(joint_values):
			joint_goal[i] = v

		success = self.active_group.go(joint_goal, wait)
		self.active_group.stop()
		return success

	def goto_pose(self, pose, velocity=1.0, group_name=None, wait=True):
		"""
		Move to pose
		:param pose: Array position & orientation [x, y, z, qx, qy, qz, qw]
		:param velocity: Velocity (fraction of max) [0.0, 1.0]
		:param group_name: Move group (use current if None)
		:param wait: Wait for completion if True
		:return: Bool success
		"""
		if group_name:
			self.set_group(group_name)
		if not self.active_group:
			raise ValueError('No active Planning Group')

		if type(pose) is list:
			pose = list_to_pose(pose)
		self.active_group.set_max_velocity_scaling_factor(velocity)
		self.active_group.set_pose_target(pose)
		success = self.active_group.go(wait=wait)
		self.active_group.stop()
		self.active_group.clear_pose_targets()
		return success

	def goto_pose_cartesian(self, pose, velocity=1.0, group_name=None, wait=True):
		"""
		Move to pose following a cartesian trajectory.
		:param pose: Array position & orientation [x, y, z, qx, qy, qz, qw]
		:param velocity: Velocity (fraction of max) [0.0, 1.0]
		:param group_name: Move group (use current if None)
		:param wait: Wait for completion if True
		:return: Bool success
		"""
		if group_name:
			self.set_group(group_name)
		if not self.active_group:
			raise ValueError('No active Planning Group')

		if type(pose) is list:
			pose = list_to_pose(pose)

		self.active_group.set_max_velocity_scaling_factor(velocity)
		(plan, fraction) = self.active_group.compute_cartesian_path(
										   [pose],   # waypoints to follow
										   0.005,    # eef_step
										   0.0)      # jump_threshold
		if fraction != 1.0:
			raise ValueError('Unable to plan entire path!')

		success = self.active_group.execute(plan, wait=wait)
		self.active_group.stop()
		self.active_group.clear_pose_targets()
		return success

	def goto_named_pose(self, pose_name, velocity=1.0, group_name=None, wait=True):
		"""
		Move to named pos
		:param pose: Name of named pose
		:param velocity: Velocity (fraction of max) [0.0, 1.0]
		:param group_name: Move group (use current if None)
		:param wait: Wait for completion if True
		:return: Bool success
		"""
		if group_name:
			self.set_group(group_name)
		if not self.active_group:
			raise ValueError('No active Planning Group')

		self.active_group.set_max_velocity_scaling_factor(velocity)
		self.active_group.set_named_target(pose_name)
		success = self.active_group.go(wait=wait)
		self.active_group.stop()
		return success

	def home_gripper(self):
		"""
		Home and initialise the gripper
		:return: Bool success
		"""
		client = actionlib.SimpleActionClient('franka_gripper/homing', franka_gripper.msg.HomingAction)
		client.wait_for_server()
		client.send_goal(franka_gripper.msg.HomingGoal())
		return client.wait_for_result()

	def stop_gripper(self):
		"""
		Abort gripper action
		:return: Bool success
		"""
		client = actionlib.SimpleActionClient('franka_gripper/stop', franka_gripper.msg.StopAction)
		client.wait_for_server()
		client.send_goal(franka_gripper.msg.StopGoal())
		return client.wait_for_result()

	def set_gripper(self, width, speed=0.1, wait=True):
		"""
		Set gripper with.
		:param width: Width in metres
		:param speed: Move velocity (m/s)
		:param wait: Wait for completion if True
		:return: Bool success
		"""
		client = actionlib.SimpleActionClient('franka_gripper/move', franka_gripper.msg.MoveAction)
		client.wait_for_server()
		client.send_goal(franka_gripper.msg.MoveGoal(width, speed))
		if wait:
			return client.wait_for_result()
		else:
			return True

	def grasp(self, width=0, e_inner=0.1, e_outer=0.1, speed=0.1, force=1):
		"""
		Wrapper around the franka_gripper/grasp action.
		http://docs.ros.org/kinetic/api/franka_gripper/html/action/Grasp.html
		:param width: Width (m) to grip
		:param e_inner: epsilon inner
		:param e_outer: epsilon outer
		:param speed: Move velocity (m/s)
		:param force: Force to apply (N)
		:return: Bool success
		"""
		client = actionlib.SimpleActionClient('franka_gripper/grasp', franka_gripper.msg.GraspAction)
		client.wait_for_server()
		client.send_goal(
			franka_gripper.msg.GraspGoal(
				width,
				franka_gripper.msg.GraspEpsilon(e_inner, e_outer),
				speed,
				force
			)
		)
		return client.wait_for_result()

	def stop(self):
		"""
		Stop the current movement.
		"""
		if self.active_group:
			self.active_group.stop()

	def recover(self):
		"""
		Call the error reset action server.
		"""
		self.reset_publisher.publish(ErrorRecoveryActionGoal())
		rospy.sleep(3.0)
