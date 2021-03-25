#!/usr/bin/python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose
from std_msgs.msg import ColorRGBA


class marker_organizer():

    def __init__(self, occuranceThresh, distThresh):
        rospy.init_node('face_markers_node')
        self.subscriber = rospy.Subscriber("face_pose", Pose, self.new_detection)
        self.buffer = []  # buffer to catch poses from face_pose topic
        self.publisher = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)
        self.faces = []
        self.marker_array = MarkerArray()
        self.marker_num = 1
        self.occuranceThresh = occuranceThresh
        self.distThresh = distThresh

    def new_detection(self, pose):
        print("got pose")
        self.buffer.append(pose)

    def check_faces(self):

        for pose in self.buffer:
            noMatch = 0

            for i, (face, occurances) in enumerate(self.faces):
                numMatches = 0
                if face.position.x - self.distThresh <= pose.position.x \
                        <= face.position.x + self.distThresh:
                    numMatches += 1
                if face.position.y - self.distThresh <= pose.position.y \
                        <= face.position.y + self.distThresh:
                    numMatches += 1
                # zna bit problem če so dve sliki na isti steni ampak na drugi strani

                if numMatches == 2:
                    face.position.x = (face.position.x * occurances
                                       + pose.position.x) / (occurances + 1)
                    face.position.y = (face.position.y * occurances
                                       + pose.position.y) / (occurances + 1)
                    occurances += 1
                    self.faces[i] = (face, occurances)
                    if occurances == self.occuranceThresh:
                        if self.check_clusters(i):
                            self.update_markers()
                else:
                    noMatch += 1

            if noMatch == len(self.faces):
                self.faces.append((pose, 1))
        if len(self.faces) > 5:
            rospy.logwarn("NOVO!!!")
            for a in self.faces:
                print(a[0], a[1])
        self.buffer = []

    def check_clusters(self, ix):
        (pose, occ) = self.faces[ix]
        for i, (face, occurances) in enumerate(self.faces):
            if i == ix:
                continue
            numMatches = 0
            if face.position.x - self.distThresh <= pose.position.x \
                    <= face.position.x + self.distThresh:
                numMatches += 1
            if face.position.y - self.distThresh <= pose.position.y \
                    <= face.position.y + self.distThresh:
                numMatches += 1

            if numMatches == 2:
                face.position.x = (face.position.x * occurances
                                   + pose.position.x * occ) / (occurances + occ)
                face.position.y = (face.position.y * occurances
                                   + pose.position.y * occ) / (occurances + occ)
                occurances += occ
                self.faces[i] = (face, occurances)
                return False
        return True

    def update_markers(self):
        self.marker_array.markers = []
        self.marker_num = 1
        for (pose, occurances) in self.faces:
            if occurances >= self.occuranceThresh:
                self.marker_array.markers.append(self.make_marker(pose))
                self.marker_num += 1
        self.publisher.publish(self.marker_array)
        print("Marker addded")

    def make_marker(self, pose):
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose = pose
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.id = self.marker_num
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0, 1, 0, 1)
        return marker


def main():
    marker_org = marker_organizer(5, 0.5)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker_org.check_faces()
        rate.sleep()


if __name__ == "__main__":
    main()
