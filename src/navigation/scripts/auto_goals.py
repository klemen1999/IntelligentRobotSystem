import rospy
import os
import math
import cv2
import actionlib
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import *
from PIL import Image
from skimage.morphology import skeletonize
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Pose, Quaternion
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
    def __init__(self, gridSize, viz=False):
        self.gridSize = gridSize
        self.viz = viz
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

        kernel = np.ones((10, 10), np.uint8)
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
            pathPoints.append(maxP)

        betterPoints = []
        pathPoints.reverse()  # just some hardcoding to get the correct points
        distThreshold = 10  # how close can points be together
        add = True
        for i in range(0, len(pathPoints)): # removing to close points
            for j in range(i + 1, len(pathPoints)):
                dist = np.linalg.norm(pathPoints[i] - pathPoints[j])
                if dist < distThreshold:
                    add = False
                    break
            if add:
                betterPoints.append(pathPoints[i])
            add = True

        if self.viz:
            self.vizualize(image, path, betterPoints)

        return betterPoints

    def vizualize(self, image, path, points): # if we want to see an image with path and goals
        for point in path:
            image[point[0],point[1]] = 60
        for point in points:
            image[point[0],point[1]] = 150

        coloredImage = np.zeros((image.shape[0], image.shape[1], 3), np.uint8)
        coloredImage[image == 255] = [255, 255, 255]
        coloredImage[image == 60] = [42, 245, 86]
        coloredImage[image == 205] = [40, 40, 40]
        coloredImage[image == 150] = [245, 59, 42]
        Image.fromarray(coloredImage).show()


    def transform_points(self, points): #transforming points to map coordinates
        transPoints = []
        for point in points:
            pt = PoseStamped()
            pt.pose.position.x = point[1] * self.map.res
            pt.pose.position.y = (self.map.height - point[0]) * self.map.res
            pt_t = tf2_geometry_msgs.do_transform_pose(pt, self.map.transform)
            transPoints.append(pt_t.pose)
        return transPoints


    def calcDistance(self, point1, point2):
        return math.sqrt((point1.x-point2.x)**2 + (point1.y-point2.y)**2)

    def findClosest(self, current, pointList):
        minD = np.inf
        minP = None
        for point in pointList:
            dist = self.calcDistance(current.position, point.position)
            if dist<minD:
                minD = dist
                minP = point
        pointList.remove(minP)
        return minP, pointList

    def sortPoints(self, points): # finding an optimal path through the goals
        sortedPoints = []
        prev = Pose()
        while len(points) > 0:
            current, points = self.findClosest(prev, points)
            sortedPoints.append(current)
            prev = current

        return sortedPoints

    def look_at(self, curr_pose, target_pose): # calc pose at curr_pose position looking at target_pose
        new = Pose()
        dx = target_pose.position.x - curr_pose.position.x
        dy = target_pose.position.y - curr_pose.position.y
        rad = math.atan2(dy, dx)
        q = quaternion_from_euler(0, 0, rad)
        q = Quaternion(q[0], q[1], q[2], q[3])
        new.position = curr_pose.position
        new.orientation = q
        return new

    def getPoses(self, points): # points to poses, ready for move_base
        poses = []
        for i in range(0,len(points)):
            if i == (len(points)-1):
                pose = self.look_at(points[i], points[0])
            else:
                pose = self.look_at(points[i], points[i+1])
            poses.append((pose, 300, 1))
        return poses


    def get_mapGoals(self):
        points = self.get_goalsImage()
        transPoints = self.transform_points(points)
        sortedPoints = self.sortPoints(transPoints)
        poses = self.getPoses(sortedPoints)
        return poses


def move_points(points):
    client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()

    for (point, _, _) in points:
        p = PoseStamped()
        p.pose = point
        move(p, client)
        print("going to next goal")
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
    navigator = AutoNav(33, True)
    points = navigator.get_mapGoals()
    move_points(points)
