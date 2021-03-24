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
from geometry_msgs.msg import PointStamped, Vector3, Pose, Twist


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

		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_rotation)
		self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
		self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		self.points = points
		self.facePose_sub = rospy.Subscriber("face_pose", Pose, self.new_detection)

		self.slowDown = False
		self.slowDownStart = None
		self.slowDownDur = rospy.Duration(3)


	def print_status(self, data):
		if len(data.status_list) < 1:
			return
		output = "Status: " + str(data.status_list[-1].status) + " Text: " + str(data.status_list[-1].text)
		if data.status_list[-1].status == 3:
			rospy.loginfo(output)
		elif data.status_list[-1].status > 3:
			rospy.logwarn(output)

	def move_to_points(self):
		client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		rospy.loginfo("Waiting for move base server")
		client.wait_for_server()

		for i in range(len(self.points)):
			x, y = self.points[i]
			self.move(x, y, client)
			self.rotate(30, 360, True)


		rospy.loginfo("End")

	def move(self, x, y, client):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		goal.target_pose.pose.orientation.w = 1

		client.send_goal(goal)
		client.wait_for_result()

	def new_detection(self, pose):
		if not self.slowDown:
			print("New possible detection")
			self.slowDownStart = rospy.Time.now()
			self.slowDown = True

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
			if self.slowDownStart is not None and \
					rospy.Time.now() - self.slowDownStart > self.slowDownDur:
				print("Slow down ended by time limit")
				self.slowDownStart = None
				self.slowDown = False

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
		if self.slowDown:
			return angular_speed * 0
		else:
			return angular_speed

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


def main():
	points = readPoints()
	mover = move_controller(points, False)
	mover.move_to_points()


if __name__ == "__main__":
	main()
