#! /usr/bin/env python

import rospy
import os
import actionlib
import tf2_ros
import json
from math import pi
from nav_msgs.msg import Odometry
from tf.transformations import *
from tf import LookupException, ConnectivityException
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PointStamped, Vector3, Pose, Twist, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from face_detection.msg import ImageStatus
from sound.msg import RobotSpeakRequest



def readPoints():
	points = []
	path = os.path.realpath(__file__)
	pointsPath = os.path.join(os.path.dirname(os.path.dirname(path)), "goals/coordinates.txt")
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

def read_points_json():
	points = []
	path = os.path.realpath(__file__)
	pointsPath = os.path.join(os.path.dirname(os.path.dirname(path)), "goals/coordinates_json.json")
	with open(pointsPath, 'r') as f:
		lines = f.read()
		json_array = json.load(lines)
		for json_obj in json_array:
			self.points.append(Point.init_from_json(json_obj))

class move_controller():

	def __init__(self, points, debugStauts):
		rospy.init_node("move_robot_node")

		self.marker_sub = rospy.Subscriber('face_markers', MarkerArray, self.marker_recieved)
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_rotation)
		self.goal_status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.goal_status_callback)
		self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
		self.sound_pub = rospy.Publisher('/robot_say', RobotSpeakRequest, queue_size = 10)
		self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		self.points = points
		self.facePose_sub = rospy.Subscriber("face_pose", Pose, self.new_detection)
		self.face_status_sub = rospy.Subscriber("face_status", ImageStatus, self.new_face_detection)

		self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		rospy.loginfo("Waiting for move base server")
		self.client.wait_for_server()

		self.moving = False
		self.slowDown = False
		self.currentMove = None
		self.lastPopped = None
		self.slowDownStart = None
		self.face_marker_array = None
		self.slowDownDur = rospy.Duration(3)
		self.distance_to_face = 0.45

		self.alreadyVisitedMarkers = []

	def goal_status_callback(self, data):
		if len(data.status_list) >= 1:
			status = data.status_list[-1].status 
			if status == 3 and self.currentMove != None and self.currentMove != self.lastPopped:

				print("popping status")
				self.lastPopped = self.points.pop(0)


	def move_to_points(self):

		while len(self.points) > 0:
			pose, rotDeg, clockwise = self.points[0]
			self.currentMove = self.points[0]
			self.move(pose)
			self.rotate(30, rotDeg, clockwise)
			#try to move to a face (if you found new/haven't visited already)
			try:
				if self.face_marker_array != None:
					self.move_to_faces()
			except Exception as e:
				print(e)

		rospy.loginfo("End")

	def move_to_faces(self):
		for marker in self.face_marker_array:
			#check if you already visited the marker
			if marker.id not in self.alreadyVisitedMarkers:
				pose = self.approach_transform(self.current_position, marker.pose)
				self.move(pose)
				sentence = RobotSpeakRequest()
				sentence.message = "Hello face"
				self.sound_pub.publish(sentence)
				rospy.sleep(2)
				self.alreadyVisitedMarkers.append(marker.id)	#add marker id you already visited


	def move(self, pose):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.pose = pose

		self.moving = True
		self.client.send_goal(goal)
		self.client.wait_for_result()
		self.moving = False

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
		if (self.slowDownStart is not None and \
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
		self.face_marker_array = msg.markers
		f_pose = self.approach_transform(self.current_position, msg.markers[-1].pose)
		if self.moving:
			self.client.cancel_goal()
			self.points.insert(0,(f_pose, 0, True))
		print(f_pose)
		print("Marker added ", msg)

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


class Point:

	def __init(self, x, y, z, w):
		self.x = x
		self.y = y
		self.z = z
		self.w = w

	@staticmethod
	def init_from_json(self, json_obj):
		self.x = json_obj["x"]
		self.y = json_obj["y"]
		if 'z' in json_obj:
			self.z = json_obj["z"]
		if 'w' in json_obj:
			self.w = json_obj["w"]


def main():
	points = readPoints()
	mover = move_controller(points, False)
	mover.move_to_points()

if __name__ == "__main__":
	main()
