#!/usr/bin/python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose, Point
from std_msgs.msg import ColorRGBA
from face_detection.msg import ImageStatus, FacePoses
from face_detection.srv import FaceNormal
from nav_msgs.msg import Odometry
from navigation.msg import CalibrationMsg
import numpy as np
import math


class marker_organizer():

    def __init__(self, occuranceThresh, distThresh):
        rospy.init_node('face_markers_node')
        self.subscriber = rospy.Subscriber("face_pose", FacePoses, self.new_detection)
        self.publisher = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)
        self.img_status_pub = rospy.Publisher('face_status', ImageStatus, queue_size=10)
        self.calibration_sub = rospy.Subscriber("calibration_status", CalibrationMsg, self.calibration_callback)
        self.normal_srv = rospy.Service("face_normal", FaceNormal, self.get_normal)
        self.buffer = []  # buffer to catch poses from face_pose topic
        self.faces = []
        self.marker_array = MarkerArray()
        self.markerID = 1
        self.occuranceThresh = occuranceThresh
        self.distThresh = distThresh

        self.start = False # TODO: CHANGE TO FALSE


    def calibration_callback(self, msg):
        if msg.calibrationFinished:
            print("Starting with detection")
            self.start = True

    def new_detection(self, pose):
        if self.start:
            self.update_markers()
            self.buffer.append(pose)


    # self.faces has tuple of (poseMiddle, normal, occurances, markerID)
    def check_faces(self):
        for pose in self.buffer:
            noMatch = 0

            poseMiddle = pose.poseMiddle
            poseLeft = pose.poseLeft
            poseRight = pose.poseRight
            # calculating new unit normal based on new poses
            vecLeftRight = np.array([poseRight.position.x - poseLeft.position.x,
                                     poseRight.position.y - poseLeft.position.y])
            newNormal = np.array([vecLeftRight[1], -vecLeftRight[0]])
            newUnitNormal = newNormal / np.linalg.norm(newNormal)

            for i, (face, normal, occurances, markerID) in enumerate(self.faces):
                numMatches = 0
                # check for x and y
                if face.position.x - self.distThresh <= poseMiddle.position.x \
                        <= face.position.x + self.distThresh:
                    numMatches += 1
                if face.position.y - self.distThresh <= poseMiddle.position.y \
                        <= face.position.y + self.distThresh:
                    numMatches += 1
                # check for normal matching: greater than 0 means angle is >90
                if np.dot(normal, newUnitNormal) > 0:
                    numMatches += 1
                # we have new detection of known face
                if numMatches == 3:
                        face.position.x = (face.position.x * occurances
                                           + poseMiddle.position.x) / (occurances + 1)
                        face.position.y = (face.position.y * occurances
                                           + poseMiddle.position.y) / (occurances + 1)
                        normal = (normal * occurances + newUnitNormal) / (occurances + 1)
                        occurances += 1
                        self.faces[i] = (face, normal, occurances, markerID)

                else:  # didn't match on all -> could be new face
                    noMatch += 1

            if noMatch == len(self.faces):  # definetly new face
                print("Possible new face")
                status_message = ImageStatus()
                status_message.status = "NEW_FACE"
                newId = self.markerID
                self.img_status_pub.publish(status_message)
                self.faces.append((poseMiddle, newUnitNormal, 1, newId))
                self.markerID += 1
        self.buffer = []

    def update_markers(self):
        self.marker_array.markers = []
        for (pose, _, occurances, markerID) in self.faces:
            self.marker_array.markers.append(self.make_marker(pose, occurances, markerID))

        self.publisher.publish(self.marker_array)
        print("Markes updated")

    def make_marker(self, pose, occurances, markerID):
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose = pose
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = str(occurances)
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.id = markerID
        marker.scale = Vector3(0.3,0.3,0.3)
        marker.color = ColorRGBA(0, 1, 0, 1) if occurances >= self.occuranceThresh \
            else ColorRGBA(1, 0, 0, 1)
        return marker

    def get_normal(self, request):
        print(request)
        for (_, normal, _, markerID) in self.faces:
            if request.markerID == markerID:
                msg = FaceNormal()
                msg.unitNormal = np.copy(normal)
                return msg

def main():
    marker_org = marker_organizer(5, 0.5)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker_org.check_faces()

        rate.sleep()


if __name__ == "__main__":
    main()
