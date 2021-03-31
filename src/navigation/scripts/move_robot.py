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
from std_msgs.msg import ColorRGBA
from sound.msg import RobotSpeakRequest
from navigation.msg import CalibrationMsg
from nav_msgs.srv import GetPlan


def readPoints():
	points = []
	path = os.path.realpath(__file__)
	pointsPath = os.path.join(os.path.dirname(os.path.dirname(path)), "goals/coordinates.txt")
	# pointsPath = os.path.join(os.path.dirname(os.path.dirname(path)), "goals/coordinates2.txt")
	f = open(pointsPath, "r")
	lines = f.readlines()
	for i in range(0, len(lines), 9):
		newPose = Pose()
		newPose.position.x = float(lines[i + 1].split(":")[1])
		newPose.position.y = float(lines[i + 2].split(":")[1])
		newPose.position.z = float(lines[i + 3].split(":")[1])
		newPose.orientation.x = float(lines[i + 4].split(":")[1])
		newPose.orientation.y = float(lines[i + 5].split(":")[1])
		newPose.orientation.z = float(lines[i + 6].split(":")[1])
		newPose.orientation.w = float(lines[i + 7].split(":")[1])
		temp = lines[i + 8].split(" ")
		rotDeg = float(temp[0])
		clockwise = True if int(temp[1]) else False
		points.append((newPose, rotDeg, clockwise))
	return points


class move_controller():

	def __init__(self, points, debugStauts=False):
		rospy.init_node("move_robot_node")
		if debugStauts:
			self.status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.print_status)
		self.marker_sub = rospy.Subscriber('face_markers', MarkerArray, self.marker_recieved)
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_rotation)
		self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
		self.points = points
		self.facePose_sub = rospy.Subscriber("face_pose", Pose, self.new_detection)
		self.face_status_sub = rospy.Subscriber("face_status", ImageStatus, self.new_face_detection)
		self.sound_pub = rospy.Publisher("robot_say", RobotSpeakRequest, queue_size=10)

		self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		rospy.loginfo("Waiting for move base server")
		self.client.wait_for_server()

		self.slowDown = False
		self.slowDownStart = None
		self.slowDownDur = rospy.Duration(3)
		self.distance_to_face = 0.45

		self.alreadyVisitedMarkers = []

		self.goal_publisher = rospy.Publisher('goal_markers', Marker, queue_size=1)
		self.marker_num = 10000

		self.calibration_pub = rospy.Publisher("calibration_status", CalibrationMsg, queue_size=10)

		self.goal_checker = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

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

	def move_to_points(self):
		for point in self.points:
			pose, rotDeg, clockwise = point
			marker = self.make_marker(pose)
			self.goal_publisher.publish(marker)
			self.move(pose)
			self.rotate(40, rotDeg, clockwise)

			# try to move to a face (if you found new/haven't visited already)
			try:
				if self.face_marker_array != None and self.face_marker_array.markers != None and len(
						self.face_marker_array.markers) > 0:
					self.move_to_faces()
			except Exception as e:
				print(e)

			if len(self.alreadyVisitedMarkers) == 3:
				print("Found 3 faces. I'm gonna stop now.")
				return

		rospy.loginfo("End")

	def move_to_faces(self):
		for marker in self.face_marker_array.markers:
			# check if you already visited the marker
			if marker.id in self.alreadyVisitedMarkers:
				continue
			pose = self.approach_transform(self.current_position, marker.pose)
			markerToFace = self.make_marker(pose)
			self.goal_publisher.publish(markerToFace)
			self.move(pose)
			soundMsg = RobotSpeakRequest()
			soundMsg.message = "Hello"
			self.sound_pub.publish(soundMsg)
			rospy.sleep(1)
			self.alreadyVisitedMarkers.append(marker.id)  # add marker id you already visited

	def move(self, pose):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.pose = pose

		self.client.send_goal(goal)
		self.client.wait_for_result()

	def new_detection(self, pose):
		print("New possible detection")

	def new_face_detection(self, msg):
		if (msg.status == "NEW_FACE"):
			print("New face")
			self.slowDown = True
			self.slowDownStart = rospy.Time.now()

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
			vel_msg.angular.z = self.adjust_the_speed(angular_speed)
			self.velocity_pub.publish(vel_msg)

			prev_rotation = current_rotation
			current_rotation = self.current_rotation

			rotated += abs(abs(current_rotation) - abs(prev_rotation))
			rate.sleep()

		vel_msg.angular.z = 0
		self.velocity_pub.publish(vel_msg)

	def closeTo(self, value, reference, threshold):
		return (value <= (reference + threshold) and value >= reference) \
			   or (value >= (reference - threshold) and value <= reference)

	def get_rotation(self, msg):
		self.current_position = msg.pose.pose
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		self.current_rotation = yaw

	def get_angular_speed(self, angular_speed, clockwise):
		if (clockwise):
			return -abs(angular_speed)
		else:
			return abs(angular_speed)

	def adjust_the_speed(self, angular_speed):
		if self.slowDown and self.not_timed_out():
			return angular_speed * 0
		else:
			return angular_speed

	def not_timed_out(self):
		if (self.slowDownStart is not None and
				rospy.Time.now() - self.slowDownStart > self.slowDownDur):
			print("timed out")
			self.slowDown = False
			self.slowDownStart = None
			return True
		else:
			return True

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

	def marker_recieved(self, msg):
		self.slowDownStart = None
		self.slowDown = False
		self.face_marker_array = msg

	def approach_transform(self, curr_pose, target_pose):
		dx = target_pose.position.x - curr_pose.position.x
		dy = target_pose.position.y - curr_pose.position.y
		v = Vector3(dx, dy, 0)
		v_len = math.sqrt(math.pow(v.x, 2) + math.pow(v.y, 2))
		v_new_len = v_len - self.distance_to_face
		v_mul = v_new_len / v_len

		v = Vector3(v.x * v_mul, v.y * v_mul, 0)
		v = Vector3(v.x + curr_pose.position.x, v.y + curr_pose.position.y, 0)

		rad = math.atan2(dy, dx)
		q = quaternion_from_euler(0, 0, rad)
		q = Quaternion(q[0], q[1], q[2], q[3])

		pose = Pose(v, q)
		if self.check_if_reachable(curr_pose, pose):
			return pose
		else:
			pose = self.closest_avaliable(curr_pose,pose)
			pose = self.look_at(pose, target_pose)
			return pose


	def check_if_reachable(self, currPose, targetPose):
		start = PoseStamped()
		start.header.frame_id = "map"
		start.pose = currPose
		goal = PoseStamped()
		goal.header.frame_id = "map"
		goal.pose = targetPose
		tolerance = 0
		response = self.goal_checker(start, goal, tolerance)
		if len(response.plan.poses) > 0:
			return True
		else:
			return False

	def closest_avaliable(self, currPose, targetPose):
		tempRight = tempLeft = tempUp = tempDown = targetPose
		margin = 0.2
		counter = 0
		while counter < 10:
			tempRight.position.x += margin
			if self.check_if_reachable(currPose, tempRight):
				print("right")
				return tempRight
			tempLeft.position.x -= margin
			if self.check_if_reachable(currPose, tempLeft):
				print("left")
				return tempLeft
			tempLeft.position.y += margin
			if self.check_if_reachable(currPose, tempUp):
				print("up")
				return tempUp
			tempLeft.position.y -= margin
			if self.check_if_reachable(currPose, tempDown):
				print("down")
				return tempDown
			counter += 1
		print("Didn't find any close backup point")
		return False

	def look_at(self, curr_pose, target_pose):
		dx = target_pose.position.x - curr_pose.position.x
		dy = target_pose.position.y - curr_pose.position.y
		rad = math.atan2(dy, dx)
		q = quaternion_from_euler(0, 0, rad)
		q = Quaternion(q[0], q[1], q[2], q[3])
		target_pose.orientation = q
		return target_pose

	def ghetto_move(self):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.pose.position.x = -0.17239
		goal.target_pose.pose.position.y = -0.8683148
		goal.target_pose.pose.orientation.z = -0.63146085
		goal.target_pose.pose.orientation.w = 0.7754
		self.client.send_goal(goal)
		self.client.wait_for_result()

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
	points = readPoints()
	mover = move_controller(points, True)
	mover.calibrate()
	mover.move_to_points()


if __name__ == "__main__":
	main()
