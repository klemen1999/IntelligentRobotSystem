#!/usr/bin/python

import rospy
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose, Point
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from navigation.msg import CalibrationMsg
from cylinder_detection.msg import CylinderPoseColor
from cylinder_detection.srv import CylinderStatus, CylinderStatusResponse
import operator
import numpy as np

class marker_organizer():

    def __init__(self, occuranceThresh, distThresh):
        rospy.init_node('cylinder_markers_node')
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_odometry)
        self.subscriber = rospy.Subscriber("cylinder_pose_color", CylinderPoseColor, self.new_detection)
        self.publisher = rospy.Publisher('cylinder_markers', MarkerArray, queue_size=1000)
        self.calibration_sub = rospy.Subscriber("calibration_status", CalibrationMsg, self.calibration_callback)
        self.status_srv = rospy.Service("cylinder_status", CylinderStatus, self.get_status)
        self.buffer = []  # buffer to catch poses from cylinder_pose topic
        self.cylinders = []
        self.marker_array = MarkerArray()
        self.markerID = 1
        self.occuranceThresh = occuranceThresh
        self.distThresh = distThresh
        self.col_dict = {"white": ColorRGBA(255 / 255, 255 / 255, 255 / 255, 1), "black": ColorRGBA(0, 0, 0, 1),
                    "red": ColorRGBA(255 / 255, 0, 0, 1),
                    "blue": ColorRGBA(0, 0, 255 / 255, 1), "green": ColorRGBA(0, 255 / 255, 0, 1),
                    "yellow": ColorRGBA(247 / 255, 202 / 255, 24 / 255, 1)}
        self.start = True  #TODO: CHANGE THIS TO FALSE

    def get_odometry(self, msg):
        self.current_position = msg.pose.pose

    def calibration_callback(self, msg):
        if msg.calibrationFinished:
            print("Starting with detection")
            self.start = True

    def new_detection(self, pose):
        if self.start:
            self.buffer.append(pose)

    # self.cylinder has tuple of (poseMiddle, occurances, colors dictionary, markerID)
    def check_cylinders(self):
        for pose in self.buffer:
            noMatch = 0

            poseMiddle = pose.pose
            newColor = pose.color
            colorWeight = self.calc_color_weight(poseMiddle, self.current_position)

            for i, (cylinder, occurances, colors, markerID) in enumerate(self.cylinders):
                numMatches = 0
                # check for x and y
                if cylinder.position.x - self.distThresh <= poseMiddle.position.x \
                        <= cylinder.position.x + self.distThresh:
                    numMatches += 1
                if cylinder.position.y - self.distThresh <= poseMiddle.position.y \
                        <= cylinder.position.y + self.distThresh:
                    numMatches += 1

                # we have new detection of known cylinder
                if numMatches == 2:
                    cylinder.position.x = (cylinder.position.x * occurances
                                       + poseMiddle.position.x) / (occurances + 1)
                    cylinder.position.y = (cylinder.position.y * occurances
                                       + poseMiddle.position.y) / (occurances + 1)
                    occurances += 1
                    if newColor not in colors:
                        colors[newColor] = 1*colorWeight
                    else:
                        colors[newColor] += (1+colorWeight)
                    self.cylinders[i] = (cylinder, occurances, colors, markerID)

                else:  # didn't match on all -> could be new cylinder
                    noMatch += 1

            if noMatch == len(self.cylinders):  # definetly new cylinder
                print("Possible new cylinder")
                newId = self.markerID
                color = {newColor: (1*colorWeight)}
                self.cylinders.append((poseMiddle, 1, color, newId))
                self.markerID += 1
        self.buffer = []

    def calc_color_weight(self, start, end):
        dist = math.sqrt((start.position.x-end.position.x)**2+(start.position.y-end.position.y)**2)
        return 1/dist

    def update_markers(self):
        self.marker_array.markers = []
        for (pose, occurances, colors, markerID) in self.cylinders:
            self.marker_array.markers.append(self.make_marker(pose, occurances, colors, markerID))

        self.publisher.publish(self.marker_array)
        print("Markes updated")

    def make_marker(self, pose, occurances, colors, markerID):
        currentColor = max(colors.items(), key=operator.itemgetter(1))[0] #get key of color that occures most
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose = pose
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = "C:"+str(occurances)
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.id = markerID
        marker.scale = Vector3(0.3,0.3,0.3)
        marker.color = self.col_dict[currentColor] if occurances >= self.occuranceThresh else self.col_dict["white"]

        return marker

    def get_status(self, request):
        print("Got status request for marker id:", request.markerID)
        for (_, occurances, colors, markerID) in self.cylinders:
            if request.markerID == markerID:
                msg = CylinderStatusResponse()
                msg.viable = True if occurances >= self.occuranceThresh else False
                msg.color = max(colors.items(), key=operator.itemgetter(1))[0]
                return msg

def main():
    marker_org = marker_organizer(2, 0.5)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker_org.check_cylinders()
        marker_org.update_markers()
        rate.sleep()


if __name__ == "__main__":
    main()
