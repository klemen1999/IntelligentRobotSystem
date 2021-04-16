import rospy
import os
import cv2
import actionlib
import numpy as np
import tf2_ros
import tf2_geometry_msgs
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


def get_goals():
    path = os.path.dirname(os.path.realpath(__file__))
    parent = os.path.abspath(os.path.join(path, os.pardir))
    extension = "/images/map.pgm"
    imagePath = parent+extension

    img = cv2.imread(imagePath, 0)

    image = np.copy(img) # map with 0 and 255 only
    image[image < 240] = 0
    image[image >= 240] = 255

    skeleton = skeletonize(image, method="lee") # skeletonized map
    skeleton[skeleton == 255] = 60
    path = np.argwhere(skeleton == 60) # pixels that are part of the path

    gridPoints = [] # grid inside the map where distance between is 25
    for i in range(0, img.shape[0], 25):
        for j in range(0, img.shape[1], 25):
            if (i == 0 or i == img.shape[0] - 1
                    or j == 0 or j == img.shape[1] - 1):
                continue

            if not (img[i + 1, j] == 205 and img[i - 1, j] == 205
                    and img[i, j + 1] == 205 and img[i, j - 1] == 205):
                gridPoints.append((i, j))

    pathPoints = [] # calculating nearest path points for every grid points
    for point in gridPoints:
        maxD = np.inf
        maxP = [0, 0]
        for pathP in path:
            dist = np.linalg.norm(point - pathP)
            if dist < maxD:
                maxD = dist
                maxP = pathP
        pathPoints.append((maxP[0],maxP[1]))

    return pathPoints

def transform_points(points, map):
    transPoints = []
    for point in points:
        pt = PoseStamped()
        pt_t = PoseStamped()
        pt.pose.position.x = point[1] * map.res
        
        pt.pose.position.y = (map.height - point[0]) * map.res

        pt_t = tf2_geometry_msgs.do_transform_pose(pt,map.transform)

        transPoints.append(pt_t)


    return transPoints


def move_points(points):
    client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()

    for point in points:
        move(point, client)

    rospy.loginfo("End")


def move(point, client):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose = point.pose
    goal.target_pose.pose.orientation.w = 1
    goal.target_pose.header.stamp = rospy.Time.now()
    print(goal)
    client.send_goal(goal)
    client.wait_for_result()


if __name__ == '__main__':
    rospy.init_node("goals_node")
    mapData = rospy.wait_for_message("map", OccupancyGrid)

    map = Map(mapData)

    points = get_goals()

    transPoints = transform_points(points,map)

    move_points(transPoints)
