#!/usr/bin/python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose, Point
from std_msgs.msg import ColorRGBA
from face_detection.msg import ImageStatus, FacePoses, FaceDetection, FacesList
from face_detection.srv import FaceNormal, FaceNormalResponse, FaceMask, FaceMaskResponse
from nav_msgs.msg import Odometry
from navigation.msg import CalibrationMsg
import numpy as np
import math


class Face():
    def __init__(self, poseMiddle, normal, markerID, hasMask, maskWeight):
        self.pose = poseMiddle
        self.normal = normal
        self.occurances = 1
        self.maskArray = np.array([1*maskWeight,0]) if hasMask else np.array([0,1*maskWeight])
        self.markerID = markerID


class marker_organizer():

    def __init__(self, occuranceThresh, distThresh):
        rospy.init_node('face_markers_node')
        self.subscriber = rospy.Subscriber("face_pose", FacePoses, self.new_detection)
        self.publisher = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)
        self.detection_pub = rospy.Publisher("face_detection", FacesList, queue_size=10)  # list of detections
        self.img_status_pub = rospy.Publisher('face_status', ImageStatus, queue_size=10)
        self.calibration_sub = rospy.Subscriber("calibration_status", CalibrationMsg, self.calibration_callback)
        self.normal_srv = rospy.Service("face_normal", FaceNormal, self.get_normal)
        self.mask_srv = rospy.Service("face_mask", FaceMask, self.get_mask)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_odometry)
        self.buffer = []  # buffer to catch poses from face_pose topic
        self.faces = []
        self.marker_array = MarkerArray()
        self.markerID = 1
        self.occuranceThresh = occuranceThresh
        self.distThresh = distThresh
        self.start = True  # TODO: CHANGE TO FALSE

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


    def check_faces(self):
        for pose in self.buffer:
            noMatch = 0

            poseMiddle = pose.poseMiddle
            poseLeft = pose.poseLeft
            poseRight = pose.poseRight
            hasMask = pose.hasMask
            # calculating new unit normal based on new poses
            vecLeftRight = np.array([poseRight.position.x - poseLeft.position.x,
                                     poseRight.position.y - poseLeft.position.y])
            newNormal = np.array([vecLeftRight[1], -vecLeftRight[0]])
            newUnitNormal = newNormal / np.linalg.norm(newNormal)
            maskWeight = self.calc_mask_weight(poseMiddle, self.current_position) 
            gotMatch = False
            faceMatched = None
            distanceMatched = 10000000
            for i, face in enumerate(self.faces):
                numMatches = 0
                # check for x and y
                if face.pose.position.x - self.distThresh <= poseMiddle.position.x \
                        <= face.pose.position.x + self.distThresh:
                    numMatches += 1
                if face.pose.position.y - self.distThresh <= poseMiddle.position.y \
                        <= face.pose.position.y + self.distThresh:
                    numMatches += 1
                # check for normal matching: greater than 0 means angle is >90
                if np.dot(face.normal, newUnitNormal) > 0:
                    numMatches += 1
                # we have new detection of known face
                if numMatches == 3:
                    #izracunaj za koliko se popravi marker (razlika x + razlika y)
                    razlika = (
                            abs(face.pose.position.x - ((face.pose.position.x * face.occurances
                                + poseMiddle.position.x) / (face.occurances + 1))) 
                                + 
                            abs(face.pose.position.y - (face.pose.position.y * face.occurances
                                + poseMiddle.position.y) / (face.occurances + 1))
                            )
                    """
                    # ne tukaj popravljat ker potem lahko pride v več kot en cluster
                        face.pose.position.x = (face.pose.position.x * face.occurances
                                           + poseMiddle.position.x) / (face.occurances + 1)
                        face.pose.position.y = (face.pose.position.y * face.occurances
                                           + poseMiddle.position.y) / (face.occurances + 1)
                        face.normal = (face.normal * face.occurances + newUnitNormal) / \
                                      (face.occurances + 1)
                        face.occurances += 1
                        
                        if hasMask:
                            face.maskArray[0] += 1
                        else:
                            face.maskArray[1] += 1
                    """
                    gotMatch = True
                    if razlika < distanceMatched:
                        #print("better match face")
                        faceMatched = face
                        distanceMatched = razlika


                else:  # didn't match on all -> could be new face
                    noMatch += 1
            if gotMatch:
                # popravi obstojec marker
                faceMatched.pose.position.x = (faceMatched.pose.position.x * faceMatched.occurances
                                           + poseMiddle.position.x) / (faceMatched.occurances + 1)
                faceMatched.pose.position.y = (faceMatched.pose.position.y * faceMatched.occurances
                                           + poseMiddle.position.y) / (faceMatched.occurances + 1)
                faceMatched.normal = (faceMatched.normal * faceMatched.occurances + newUnitNormal) / \
                                      (faceMatched.occurances + 1)
                faceMatched.occurances += 1

                   
                if hasMask:
                    faceMatched.maskArray[0] += maskWeight
                else:
                    faceMatched.maskArray[1] += maskWeight

            elif noMatch == len(self.faces):  # definetly new face
                print("Possible new face")
                status_message = ImageStatus()
                status_message.status = "NEW_FACE"
                newId = self.markerID
                self.img_status_pub.publish(status_message)
                self.faces.append(Face(poseMiddle, newUnitNormal, newId, hasMask, maskWeight))
                self.markerID += 1

        self.buffer = []

    def update_markers(self):
        self.marker_array.markers = []
        detections = FacesList()
        for face in self.faces:
            self.marker_array.markers.append(self.make_marker(face))
            element = FaceDetection()
            element.id = face.markerID
            element.pose = face.pose
            detections.list.append(element)

        self.publisher.publish(self.marker_array)
        self.detection_pub.publish(detections)
        print("Markes updated")

    def make_marker(self, face):
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose = face.pose
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = "I:"+str(face.occurances)
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.id = face.markerID
        marker.scale = Vector3(0.3,0.3,0.3)
        marker.color = ColorRGBA(0, 1, 0, 1) if face.occurances >= self.occuranceThresh \
            else ColorRGBA(1, 0, 0, 1)
        return marker
    def get_mask(self, request):
        for face in self.faces:
            if request.markerID == face.markerID:
                msg = FaceMaskResponse()
                msg.hasMask = True if face.maskArray[0] >= face.maskArray[1] else False
                return msg

    def get_normal(self, request):
        print("Got unitNormal request for marker id:", request.markerID)
        for face in self.faces:
            if request.markerID == face.markerID:
                msg = FaceNormalResponse()
                msg.unitNormal = np.copy(face.normal)
                msg.viable = True if face.occurances >= self.occuranceThresh else False
                #msg.hasMask = True if face.maskArray[0] >= face.maskArray[1] else False
                msg.warn = self.check_if_too_close(face)
                return msg

    def check_if_too_close(self, currentFace):
        for face in self.faces:
            if face != currentFace:
                dist = self.euclid_distance(face.pose.position, currentFace.pose.position)
                if dist < 1:
                    if np.dot(face.normal, currentFace.normal) > 0:
                        return True
        return False


    def euclid_distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

    def calc_mask_weight(self, start, end):
        dist = math.sqrt((start.position.x-end.position.x)**2+(start.position.y-end.position.y)**2)
        return 1/dist

def main():
    marker_org = marker_organizer(5, 0.5)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker_org.check_faces()
        rate.sleep()


if __name__ == "__main__":
    main()
