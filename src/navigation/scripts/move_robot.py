#! /usr/bin/env python

import rospy
import os
import actionlib
import tf2_ros
from math import pi
from nav_msgs.msg import Odometry
from tf.transformations import *
from tf import LookupException, ConnectivityException
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3, Pose, Twist, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from face_detection.msg import ImageStatus
from face_detection.srv import FaceNormal
from std_msgs.msg import ColorRGBA
from sound.msg import RobotSpeakRequest
from navigation.msg import CalibrationMsg
from nav_msgs.srv import GetPlan
from auto_goals import AutoNav


class move_controller():
	def __init__(self, points, debugStauts=False):
		if debugStauts:  # if we want to print status
			self.status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.print_status)

		self.points = points  # generated points
		# info about the robot position
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_odometry)
		self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
		# face subscribers
		self.marker_sub = rospy.Subscriber('face_markers', MarkerArray, self.face_marker_recieved)
		self.face_normal_client = rospy.ServiceProxy("face_normal", FaceNormal)
		# publisher for sound
		self.sound_pub = rospy.Publisher("robot_say", RobotSpeakRequest, queue_size=10)
		# move base client
		self.move_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		rospy.loginfo("Waiting for move base server")
		self.move_client.wait_for_server()
		# markers to show next goal
		self.goal_publisher = rospy.Publisher('goal_markers', Marker, queue_size=1)
		self.marker_num = 10000
		# calibration status publisher
		self.calibration_pub = rospy.Publisher("calibration_status", CalibrationMsg, queue_size=10)
		# service to check if goal is reachable
		self.goal_checker = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

		self.distance_to_face = 0.45
		self.alreadyVisitedMarkers = []

	def print_status(self, data):
		if len(data.status_list) < 1:
			return
		output = "Status: " + str(data.status_list[-1].status) + " Text: " + str(data.status_list[-1].text)
		if data.status_list[-1].status == 3:
			rospy.loginfo(output)
		elif data.status_list[-1].status > 3:
			rospy.logwarn(output)

	def calibrate(self):
		self.rotate(40, 360, True)
		msg = CalibrationMsg()
		msg.calibrationFinished = True
		self.calibration_pub.publish(msg)

	def get_odometry(self, msg):
		self.current_position = msg.pose.pose
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		self.current_rotation = yaw

	def main_loop(self):
		for point in self.points:
			pose, rotDeg, clockwise = point
			if not self.check_if_reachable(pose):
				continue
			print("Moving to next map goal")
			marker = self.make_marker(pose)
			self.goal_publisher.publish(marker)
			self.move(pose)
			print("Rotating around at the map goal")
			self.rotate(40, rotDeg, clockwise)

			# check for new face to approach
			print("Checking for faces to approach")
			self.check_face_approach()

			# if len(self.alreadyVisitedMarkers) == 3:
			# 	print("Found 3 faces. I'm gonna stop now.")
			# 	return

		rospy.loginfo("End")

	def move(self, pose):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.pose = pose
		self.move_client.send_goal(goal)
		self.move_client.wait_for_result()

	def rotate(self, speed, angle, clockwise):
		speed_rad = self.deg_to_radian(speed)
		angular_speed = self.get_angular_speed(speed_rad, clockwise)
		angle = self.deg_to_radian(angle)
		threshold = 0.05
		rate = rospy.Rate(100)

		vel_msg = self.init_vel_msg_for_rotation()

		current_rotation = self.current_rotation
		rotated = 0

		while not self.closeTo(rotated, angle, threshold):
			vel_msg.angular.z = angular_speed
			self.velocity_pub.publish(vel_msg)

			prev_rotation = current_rotation
			current_rotation = self.current_rotation

			rotated += abs(abs(current_rotation) - abs(prev_rotation))
			rate.sleep()

		vel_msg.angular.z = 0
		self.velocity_pub.publish(vel_msg)

	def closeTo(self, value, reference, threshold):
		return ((reference + threshold) >= value >= reference) \
			   or ((reference - threshold) <= value <= reference)


	def get_angular_speed(self, angular_speed, clockwise):
		if (clockwise):
			return -abs(angular_speed)
		else:
			return abs(angular_speed)

	def deg_to_radian(self, deg):
		return deg * 2 * pi / 360

	def init_vel_msg_for_rotation(self):
		vel_msg = Twist()
		vel_msg.linear.x = 0
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		return vel_msg

	def face_marker_recieved(self, msg):
		self.face_marker_array = msg

	def check_face_approach(self):
		try:
			if self.face_marker_array != None and self.face_marker_array.markers != None and \
					len(self.face_marker_array.markers) > 0:
				self.move_to_faces()
		except Exception as e:
			print("No faces detected yet")

	def move_to_faces(self):
		for marker in self.face_marker_array.markers:
			# check if you already visited the marker
			if marker.id in self.alreadyVisitedMarkers:
				continue
			request = FaceNormal()
			request.markerID = int(marker.id)
			normalVec = self.face_normal_client(request)
			pose = self.approach_transform(marker.pose, normalVec)
			markerToFace = self.make_marker(pose)
			self.goal_publisher.publish(markerToFace)
			if self.check_if_reachable(pose):
				print("Moving to approach the face")
				self.move(pose)
				soundMsg = RobotSpeakRequest()
				soundMsg.message = "Hello"
				print("Sending Hello sound")
				self.sound_pub.publish(soundMsg)
				rospy.sleep(1)
				self.alreadyVisitedMarkers.append(marker.id)  # add marker id you already visited
			else:
				print("Can't reach approach to face goal")


	# def approach_transform(self, curr_pose, target_pose):
	# 	dx = target_pose.position.x - curr_pose.position.x
	# 	dy = target_pose.position.y - curr_pose.position.y
	# 	v = Vector3(dx, dy, 0)
	# 	v_len = math.sqrt(math.pow(v.x, 2) + math.pow(v.y, 2))
	# 	v_new_len = v_len - self.distance_to_face
	# 	v_mul = v_new_len / v_len
	#
	# 	v = Vector3(v.x * v_mul, v.y * v_mul, 0)
	# 	v = Vector3(v.x + curr_pose.position.x, v.y + curr_pose.position.y, 0)
	#
	# 	rad = math.atan2(dy, dx)
	# 	q = quaternion_from_euler(0, 0, rad)
	# 	q = Quaternion(q[0], q[1], q[2], q[3])
	#
	# 	pose = Pose(v, q)
	#
	# 	return pose

	def approach_transform(self, markerPose, normalVec):
		pose = Pose()
		pose.position = Vector3(markerPose.position.x+normalVec[0], markerPose.position.y+normalVec[0], 0)
		q = self.look_at(pose, markerPose)
		pose = Pose(v, q)
		return pose

	def check_if_reachable(self, targetPose):
		start = PoseStamped()
		start.header.frame_id = "map"
		start.pose = self.current_position
		goal = PoseStamped()
		goal.header.frame_id = "map"
		goal.pose = targetPose
		tolerance = 0
		response = self.goal_checker(start, goal, tolerance)
		if len(response.plan.poses) > 0:
			return True
		else:
			return False

	def look_at(self, start_pose, end_pose):
		dx = end_pose.position.x - start_pose.position.x
		dy = end_pose.position.y - start_pose.position.y
		rad = math.atan2(dy, dx)
		q = quaternion_from_euler(0, 0, rad)
		q = Quaternion(q[0], q[1], q[2], q[3])
		return q

	def ghetto_move(self):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.pose.position.x = -0.17239
		goal.target_pose.pose.position.y = -0.8683148
		goal.target_pose.pose.orientation.z = -0.63146085
		goal.target_pose.pose.orientation.w = 0.7754
		self.move_client.send_goal(goal)
		self.move_client.wait_for_result()

	def make_marker(self, pose):
		marker = Marker()
		marker.header.stamp = rospy.Time(0)
		marker.header.frame_id = 'map'
		marker.pose = pose
		marker.type = Marker.CYLINDER
		marker.action = Marker.ADD
		marker.frame_locked = False
		marker.lifetime = rospy.Duration.from_sec(30)
		marker.id = self.marker_num
		marker.scale = Vector3(0.1, 0.1, 0.1)
		marker.color = ColorRGBA(0.73, 0.25, 0.35, 1)
		self.marker_num += 1
		return marker


def main():
	rospy.init_node("move_robot_node")
	print("Getting autoNavigator goals")
	autoNavigator = AutoNav(25)
	points = autoNavigator.get_mapGoals()
	mover = move_controller(points[3:], False)  #TODO: change points[3:] to points
	print("Calibrating")
	mover.calibrate()
	print("Going to goals")
	mover.main_loop()


if __name__ == "__main__":
	main()
