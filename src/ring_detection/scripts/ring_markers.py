#!/usr/bin/python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from navigation.msg import CalibrationMsg


class marker_organizer():

    def __init__(self, occuranceThresh, distThresh):
        rospy.init_node('ring_markers_node')
        self.subscriber = rospy.Subscriber("ring_pose", Pose, self.new_detection)
        self.buffer = []  # buffer to catch poses from ring_pose topic
        self.publisher = rospy.Publisher('ring_markers', MarkerArray, queue_size=1000)
        
        self.rings = []
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
        if self.start:
            self.update_markers()
            self.buffer.append(pose)

    def check_rings(self):
        for pose in self.buffer:
            noMatch = 0

            for i, (ring, occurances) in enumerate(self.rings):
                numMatches = 0
                if ring.position.x - self.distThresh <= pose.position.x \
                        <= ring.position.x + self.distThresh:
                    numMatches += 1
                if ring.position.y - self.distThresh <= pose.position.y \
                        <= ring.position.y + self.distThresh:
                    numMatches += 1
                # zna bit problem če so dva kroga na isti steni ampak na drugi strani

                if numMatches == 2:
                    ring.position.x = (ring.position.x * occurances
                                           + pose.position.x) / (occurances + 1)
                    ring.position.y = (ring.position.y * occurances
                                           + pose.position.y) / (occurances + 1)
                    occurances += 1
                    self.rings[i] = (ring, occurances)

                else:  # no match x,y -> new ring
                    noMatch += 1
                    print("no match")

            if noMatch == len(self.rings):
                self.rings.append((pose, 1))
        self.buffer = []

    def update_markers(self):
        self.marker_array.markers = []
        self.marker_num = 1
        for (pose, occurances) in self.rings:
            if occurances > 1:
                self.marker_array.markers.append(self.make_marker(pose, occurances))
                self.marker_num += 1
        self.publisher.publish(self.marker_array)
        print("Markers updated")

    def make_marker(self, pose, occurances):
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose = pose
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = str(occurances)
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.id = self.marker_num
        marker.scale = Vector3(0.3,0.3,0.3)
        marker.color = ColorRGBA(0, 1, 0, 1) if occurances >= self.occuranceThresh else ColorRGBA(1, 0, 0, 1)
        return marker


def main():
    marker_org = marker_organizer(10, 0.5)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker_org.check_rings()

        rate.sleep()


if __name__ == "__main__":
    main()
