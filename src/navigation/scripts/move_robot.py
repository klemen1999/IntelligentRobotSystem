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
from geometry_msgs.msg import PointStamped, Vector3, Pose, Twist, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from face_detection.msg import ImageStatus

def readPoints():
	points = []
	path = os.path.realpath(__file__)
	pointsPath = os.path.join(os.path.dirname(os.path.dirname(path)), "goals/coordinates.txt")
	f = open(pointsPath, "r")
	for line in f:
		tempArr = line.split(",")
		coords = []
		temp = tempArr[0].split(":")
		coords.append(float(temp[1]))
		temp = tempArr[1].split(":")
		temp = temp[1].split(")")
		coords.append(float(temp[0]))
		points.append((coords[0], coords[1]))

	return points


class move_controller():

	def __init__(self, points,debugStauts):
		rospy.init_node("move_robot_node")
		if debugStauts:
			self.status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.print_status)
		self.marker_sub = rospy.Subscriber('face_markers', MarkerArray, self.marker_recieved)
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_rotation)
		self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
		self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		self.points = points
		self.facePose_sub = rospy.Subscriber("face_pose", Pose, self.new_detection)
		self.face_status_sub = rospy.Subscriber("face_status", ImageStatus, self.new_face_detection)

		self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		rospy.loginfo("Waiting for move base server")
		self.client.wait_for_server()

		self.slowDown = False
		self.slowDownStart = None
		self.slowDownDur = rospy.Duration(3)
		self.distance_to_face = 0.45

	def print_status(self, data):
		if len(data.status_list) < 1:
			return
		output = "Status: " + str(data.status_list[-1].status) + " Text: " + str(data.status_list[-1].text)
		if data.status_list[-1].status == 3:
			rospy.loginfo(output)
		elif data.status_list[-1].status > 3:
			rospy.logwarn(output)

	def move_to_points(self):

		for i in range(len(self.points)):
			x, y = self.points[i]
			self.move(x, y, 0, 1)
			self.rotate(30, 360, True)


		rospy.loginfo("End")

	def move_to_faces(self):
		for marker in self.face_marker_array.markers:
			move_to = self.approach_transform(self.current_position, marker.pose)
			self.move(move_to.position.x, move_to.position.y, move_to.orientation.z, move_to.orientation.w)

	def move(self, x, y, z, w):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		goal.target_pose.pose.orientation.w = w
		goal.target_pose.pose.orientation.z = z

		self.client.send_goal(goal)
		self.client.wait_for_result()

	def new_detection(self, pose):
		print("New possible detection")

	def new_face_detection(self, msg):
		if(msg.status == "NEW_FACE"):
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
		if(self.slowDownStart is not None and \
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
		f_pose = self.approach_transform(self.current_position, msg.markers[0].pose)
		print(f_pose)
		print("Marker added ", msg)

	def approach_transform(self, curr_pose, target_pose):
		dx = target_pose.position.x - curr_pose.position.x
		dy = target_pose.position.y - curr_pose.position.y
		v = Vector3(dx, dy, 0)
		v_len = math.sqrt(math.pow(v.x, 2) + math.pow(v.y, 2))
		v_new_len = v_len - self.distance_to_face
		v_mul = v_new_len/v_len

		v = Vector3(v.x * v_mul, v.y * v_mul, 0)
		v = Vector3(v.x + curr_pose.position.x, v.y + curr_pose.position.y, 0)


		rad = math.atan2(dy, dx)
		q = quaternion_from_euler(0, 0, rad)
		q = Quaternion(q[0], q[1], q[2], q[3])
		
		pose = Pose(v, q)

		return pose

	def ghetto_move(self):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.pose.position.x = -0.17239
		goal.target_pose.pose.position.y = -0.8683148
		goal.target_pose.pose.orientation.z = -0.63146085
		goal.target_pose.pose.orientation.w = 0.7754
		self.client.send_goal(goal)
		self.client.wait_for_result()


def main():
	points = readPoints()
	mover = move_controller(points, False)
	mover.move_to_points()
	mover.move_to_faces()


if __name__ == "__main__":
	main()
