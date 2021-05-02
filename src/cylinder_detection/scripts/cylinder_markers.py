#!/usr/bin/python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from navigation.msg import CalibrationMsg
from color_recognition.msg import PoseColor
from cylinder_detection.msg import CylinderPoseColor
import operator

class marker_organizer():

    def __init__(self, occuranceThresh, distThresh):
        rospy.init_node('cylinder_markers_node')
        self.subscriber = rospy.Subscriber("cylinder_pose_color", CylinderPoseColor, self.new_detection)
        self.buffer = []  # buffer to catch poses from cylinder_pose topic
        self.publisher = rospy.Publisher('cylinder_markers', MarkerArray, queue_size=1000)
        
        self.cylinders = []
        self.marker_array = MarkerArray()
        self.marker_num = 1
        self.occuranceThresh = occuranceThresh
        self.distThresh = distThresh
        self.robot_position = []
        self.calibration_sub = rospy.Subscriber("calibration_status", CalibrationMsg, self.calibration_callback)
        #self.start = False
        self.start = True

    def calibration_callback(self, msg):
        if msg.calibrationFinished:
            print("Startin with detection")
            self.start = True

    def new_detection(self, pose):
        print("New detection", pose)
        # if self.start:
        #     self.update_markers()
        #     self.buffer.append(pose)

    def check_cylinders(self):
        for posee in self.buffer:
            pose = posee.pose
            noMatch = 0

            for i, (cylinder, occurances, colors) in enumerate(self.cylinders):
                numMatches = 0
                if cylinder.position.x - self.distThresh <= pose.position.x \
                        <= cylinder.position.x + self.distThresh:
                    numMatches += 1
                if cylinder.position.y - self.distThresh <= pose.position.y \
                        <= cylinder.position.y + self.distThresh:
                    numMatches += 1
                # zna bit problem če so dva kroga na isti steni ampak na drugi strani

                if numMatches == 2:
                    cylinder.position.x = (cylinder.position.x * occurances
                                           + pose.position.x) / (occurances + 1)
                    cylinder.position.y = (cylinder.position.y * occurances
                                           + pose.position.y) / (occurances + 1)
                    occurances += 1
                    if posee.color in colors:
                        colors[posee.color] += 1
                    else:
                        colors[posee.color] = 1
                    self.cylinders[i] = (cylinder, occurances, colors)

                else:  # no match x,y -> new cylinder
                    noMatch += 1
                    print("no match")

            if noMatch == len(self.cylinders):
                self.cylinders.append((pose, 1, {str(posee.color): 1}))
        self.buffer = []

    def update_markers(self):
        self.marker_array.markers = []
        self.marker_num = 1
        for (pose, occurances, colors) in self.cylinders:
            if occurances > 1:
                self.marker_array.markers.append(self.make_marker(pose, occurances, colors))
                self.marker_num += 1
        self.publisher.publish(self.marker_array)
        print("Markers updated")

    def make_marker(self, pose, occurances, colors):
        color = max(colors.items(), key=operator.itemgetter(1))[0] #get key of color that occures most
        col_dict = {"white": ColorRGBA(255/255, 255/255, 255/255, 1), "black": ColorRGBA(0, 0, 0, 1), "red": ColorRGBA(255/255, 0, 0, 1), 
                    "blue": ColorRGBA(0,0,255/255,1), "green": ColorRGBA(0,255/255,0,1), "yellow": ColorRGBA(247/255,202/255,24/255,1)}
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose = pose
        marker.type = Marker.CYLINDER
        marker.text = str(occurances)
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.id = self.marker_num
        marker.scale = Vector3(0.1,0.1,0.1)
        marker.color = col_dict[color]
        
        return marker


def main():
    marker_org = marker_organizer(10, 0.5)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker_org.check_cylinders()

        rate.sleep()


if __name__ == "__main__":
    main()
