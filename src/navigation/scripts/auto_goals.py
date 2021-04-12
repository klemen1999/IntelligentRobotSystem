import rospy
import os
import cv2
import numpy as np
from skimage.morphology import skeletonize


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


if __name__ == '__main__':
    rospy.init_node("goals_node")
    points = get_goals()

