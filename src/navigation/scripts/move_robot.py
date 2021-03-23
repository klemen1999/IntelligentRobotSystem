#! /usr/bin/env python

import rospy
import os
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PointStamped, Vector3, Pose


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

	def __init__(self, points):
		rospy.init_node("move_robot_node")
		self.status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.print_status)
		self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		self.points = points
		self.facePose_sub = rospy.Subscriber("face_pose", Pose, self.new_detection)
		self.slowDown = False

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
			# TODO: rotiraj se preden greš na naslednji goal
			print("SlowDown: ",self.slowDown)

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
		# nek flag ko detektiramo face med premikanjem/detektiranjem
		# TODO: kaj narediti v tem primeru
		self.slowDown = True


def main():
	points = readPoints()
	mover = move_controller(points)
	mover.move_to_points()


if __name__ == "__main__":
	main()
