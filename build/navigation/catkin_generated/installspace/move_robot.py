#!/usr/bin/env python3

import rospy
import os
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray


def readPoints():
    path = os.path.realpath(__file__)
    print(path)


def init_listener():
    rospy.init_node("move_robot_node")
    subscriber = rospy.Subscriber("/move_base/status", GoalStatusArray, print_status)


def print_status(data):
    if len(data.status_list) < 1:
        return
    print("Status: ", data.status_list[-1].status, " Text: ", data.status_list[-1].text)


def move_points(points):
    client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()

    for i in range(len(points)):
        x, y = points[i]
        move(x, y, client)

    rospy.loginfo("End")


def move(x, y, client):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1

    client.send_goal(goal)
    client.wait_for_result()


def main():
    # init_listener()
    points = readPoints()
    # array with valid goals
    points = [(-0.450000, 2.900000), (1.050000, 1.750000), (4.000001, -0.250000), (0.050000, -0.650000), (0, 0)]


# array with invalid goal
# points = [(244,213),(1.050000,1.750000),(4.000001,-0.250000),(0.050000,-0.650000),(0,0)]
# move_points(points)


if __name__ == "__main__":
    main()
