#!/usr/bin/python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose, Point
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from navigation.msg import CalibrationMsg
from ring_detection.msg import RingPoseColor, RingDetection, RingsList
from ring_detection.srv import RingVector, RingVectorResponse
import operator
import numpy as np
import math

class marker_organizer():

    def __init__(self, occuranceThresh, distThresh):
        rospy.init_node('ring_markers_node')
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_odometry)
        self.subscriber = rospy.Subscriber("ring_pose_color", RingPoseColor, self.new_detection)
        self.publisher = rospy.Publisher('ring_markers', MarkerArray, queue_size=1000)
        self.detection_pub = rospy.Publisher("ring_detection", RingsList, queue_size=10)  # list of detections
        self.calibration_sub = rospy.Subscriber("calibration_status", CalibrationMsg, self.calibration_callback)
        self.vector_srv = rospy.Service("ring_vector", RingVector, self.get_vector)
        self.buffer = []  # buffer to catch poses from ring_pose topic
        self.rings = []
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
            self.update_markers()
            self.buffer.append(pose)

    # self.rings has tuple of (poseMiddle, vector, occurances, colors dictionary, markerID)
    def check_rings(self):
        for pose in self.buffer:
            noMatch = 0

            poseMiddle = pose.poseMiddle
            poseLeft = pose.poseLeft
            poseRight = pose.poseRight
            # calculating new unit vector based on new poses
            vecLeftRight = np.array([poseRight.position.x - poseLeft.position.x,
                                     poseRight.position.y - poseLeft.position.y])
            newUnitVec = vecLeftRight / np.linalg.norm(vecLeftRight)

            newColor = pose.color
            colorWeight = self.calc_color_weight(poseMiddle, self.current_position)

            for i, (ring, vector, occurances, colors, markerID) in enumerate(self.rings):
                numMatches = 0
                # check for x and y
                if ring.position.x - self.distThresh <= poseMiddle.position.x \
                        <= ring.position.x + self.distThresh:
                    numMatches += 1
                if ring.position.y - self.distThresh <= poseMiddle.position.y \
                        <= ring.position.y + self.distThresh:
                    numMatches += 1
                # check for normal matching
                if np.dot(vector, newUnitVec) > 0.5 or np.dot(vector, newUnitVec) < -0.5:
                    numMatches += 1
                
                # we have new detection of known ring
                if numMatches == 3:
                    ring.position.x = (ring.position.x * occurances
                                       + poseMiddle.position.x) / (occurances + 1)
                    ring.position.y = (ring.position.y * occurances
                                       + poseMiddle.position.y) / (occurances + 1)
                    vector = (vector * occurances + newUnitVec) / (occurances + 1)
                    occurances += 1
                    if newColor not in colors:
                        colors[newColor] = 1 * colorWeight
                    else:
                        colors[newColor] += (1 * colorWeight)
                    self.rings[i] = (ring, vector, occurances, colors, markerID)

                else:  # didn't match on all -> could be new ring
                    noMatch += 1

            if noMatch == len(self.rings):  # definetly new ring
                print("Possible new ring")
                newId = self.markerID
                color = {newColor: (1*colorWeight)}
                self.rings.append((poseMiddle, newUnitVec, 1, color, newId))
                self.markerID += 1
        self.buffer = []

    def calc_color_weight(self, start, end):
        dist = math.sqrt((start.position.x-end.position.x)**2+(start.position.y-end.position.y)**2)
        return 1/dist

    def update_markers(self):
        self.marker_array.markers = []
        detections = RingsList()
        for (pose, _, occurances, colors, markerID) in self.rings:
            self.marker_array.markers.append(self.make_marker(pose, occurances, colors, markerID))
            element = RingDetection()
            element.id = markerID
            element.pose = pose
            element.color = max(colors.items(), key=operator.itemgetter(1))[0]
            detections.list.append(element)

        self.publisher.publish(self.marker_array)
        self.detection_pub.publish(detections)
        print("Markes updated")

    def make_marker(self, pose, occurances, colors, markerID):
        currentColor = max(colors.items(), key=operator.itemgetter(1))[0] #get key of color that occures most
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose = pose
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = "R:"+str(occurances)
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.id = markerID
        marker.scale = Vector3(0.3,0.3,0.3)
        marker.color = self.col_dict[currentColor] if occurances >= self.occuranceThresh else self.col_dict["white"]

        return marker

    def get_vector(self, request):
        print("Got unitVector request for marker id:", request.markerID)
        for (_, vector, occurances, colors, markerID) in self.rings:
            if request.markerID == markerID:
                msg = RingVectorResponse()
                msg.unitVector = np.copy(vector)
                msg.color = max(colors.items(), key=operator.itemgetter(1))[0]
                msg.viable = True if occurances >= self.occuranceThresh else False
                return msg


def main():
    marker_org = marker_organizer(1, 0.5)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker_org.check_rings()
        rate.sleep()


if __name__ == "__main__":
    main()
