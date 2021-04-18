import rospy
import os
import cv2
import actionlib
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from matplotlib import pyplot
from skimage.morphology import skeletonize
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Map:
    def __init__(self, data):
        self.width = data.info.width
        self.height = data.info.height
        self.res = round(data.info.resolution, 2)
        self.transform = self.get_mapTransform(data.info.origin)

    def get_mapTransform(self, data):
        mapT = TransformStamped()
        mapT.transform.translation.x = data.position.x
        mapT.transform.translation.y = data.position.y
        mapT.transform.translation.z = data.position.z
        mapT.transform.rotation = data.orientation
        return mapT


class AutoNav:
    def __init__(self, gridSize):
        self.gridSize = gridSize
        self.mapData = rospy.wait_for_message("map", OccupancyGrid)
        self.map = Map(self.mapData)

    def get_goalsImage(self):
        path = os.path.dirname(os.path.realpath(__file__))
        parent = os.path.abspath(os.path.join(path, os.pardir))
        extension = "/images/map.pgm"
        imagePath = parent + extension

        img = cv2.imread(imagePath, 0)

        image = np.copy(img)  # map with 0 and 255 only
        image[image < 240] = 0
        image[image >= 240] = 255

        kernel = np.ones((5, 5), np.uint8)
        eroded_image = cv2.erode(image, kernel, iterations=1)

        skeleton = skeletonize(eroded_image, method="lee")  # skeletonized map
        skeleton[skeleton == 255] = 60
        path = np.argwhere(skeleton == 60)  # pixels that are part of the path

        gridPoints = []  # grid inside the map where distance between is self.gridSize
        for i in range(0, img.shape[0], self.gridSize):
            for j in range(0, img.shape[1], self.gridSize):
                if (i == 0 or i == img.shape[0] - 1
                        or j == 0 or j == img.shape[1] - 1):
                    continue

                if not (img[i + 1, j] == 205 and img[i - 1, j] == 205
                        and img[i, j + 1] == 205 and img[i, j - 1] == 205):
                    gridPoints.append((i, j))

        pathPoints = []  # calculating nearest path points for every grid points
        for point in gridPoints:
            maxD = np.inf
            maxP = [0, 0]
            for pathP in path:
                dist = np.linalg.norm(point - pathP)
                if dist < maxD:
                    maxD = dist
                    maxP = pathP
            pathPoints.append((maxP[0], maxP[1]))

        return pathPoints

    def transform_points(self, points):
        transPoints = []
        for point in points:
            pt = PoseStamped()
            pt.pose.position.x = point[1] * self.map.res
            pt.pose.position.y = (self.map.height - point[0]) * self.map.res
            pt_t = tf2_geometry_msgs.do_transform_pose(pt, self.map.transform)
            transPoints.append((pt_t.pose, 360, 1))

        return transPoints

    def get_mapGoals(self):
        points = self.get_goalsImage()
        transPoints = self.transform_points(points)
        return transPoints


def move_points(points):
    client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()

    for (point, _, _) in points:
        p = PoseStamped()
        p.pose = point
        move(p, client)
        print("goig to next goal")
    rospy.loginfo("End")


def move(point, client):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose = point.pose
    goal.target_pose.pose.orientation.w = 1
    goal.target_pose.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    client.wait_for_result()


if __name__ == '__main__':
    rospy.init_node("goals_node")
    navigator = AutoNav(25)
    points = navigator.get_mapGoals()
    move_points(points)
