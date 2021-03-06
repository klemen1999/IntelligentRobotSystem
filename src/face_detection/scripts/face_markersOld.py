#!/usr/bin/python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose
from std_msgs.msg import ColorRGBA
from face_detection.msg import ImageStatus


class marker_organizer():

    def __init__(self, occuranceThresh, distThresh, debug=False):
        rospy.init_node('face_markers_node')
        self.subscriber = rospy.Subscriber("face_pose", Pose, self.new_detection)
        self.buffer = []  # buffer to catch poses from face_pose topic
        self.publisher = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)
        self.img_status_pub = rospy.Publisher('face_status', ImageStatus, queue_size=10)
        self.faces = []
        self.marker_array = MarkerArray()
        self.marker_num = 1
        self.occuranceThresh = occuranceThresh
        self.distThresh = distThresh
        self.debug = debug

    def new_detection(self, pose):
        if self.debug:
            out = ""
            for (f,c) in self.faces:
                out += "[x:"+str(f.position.x)+" y:"+str(f.position.y)
                out += " occ:"+str(c)+"] "
            print("Len:", len(self.faces),out)
        else:
            print("got pose")
        self.buffer.append(pose)

    def check_faces(self):
        for pose in self.buffer:
            noMatch = 0
            for i, (face, occurances) in enumerate(self.faces):
                if self.in_threshold(face, pose):
                    face.position.x = (face.position.x * occurances
                                       + pose.position.x) / (occurances + 1)
                    face.position.y = (face.position.y * occurances
                                       + pose.position.y) / (occurances + 1)
                    occurances += 1
                    self.faces[i] = (face, occurances)
                    self.check_clusters()
                    if occurances == self.occuranceThresh:
                        self.update_markers()
                else:
                    noMatch += 1
            if noMatch == len(self.faces):
                status_message = ImageStatus()
                status_message.status = "NEW_FACE"
                self.img_status_pub.publish(status_message)
                self.faces.append((pose, 1))
        self.buffer = []

    def check_clusters(self):
        merged = True
        while merged:
            merged = self.merge_clusters()


    def merge_clusters(self):
        for i in range(len(self.faces)):
            for j in range(len(self.faces)):
                if i == j:
                    continue
                (f1, occ1) = self.faces[i]
                (f2, occ2) = self.faces[j]
                if self.in_threshold(f1, f2):
                    occ = occ1 + occ2
                    face = Pose()
                    face.position.x = (f1.position.x * occ1 + f2.position.x * occ2) / occ
                    face.position.y = (f1.position.y * occ1 + f2.position.y * occ2) / occ
                    del self.faces[i]
                    del self.faces[j]
                    self.faces.append((face, occ))
                    return True
        return False

    # TODO: problem ??e so obraza na isti steni ampak na drugi strani
    def in_threshold(self, face1, face2):
        numMatches = 0
        if face1.position.x - self.distThresh <= face2.position.x \
                <= face1.position.x + self.distThresh:
            numMatches += 1
        if face1.position.y - self.distThresh <= face2.position.y \
                <= face1.position.y + self.distThresh:
            numMatches += 1
        return True if numMatches == 2 else False

    def update_markers(self):
        self.marker_array.markers = []
        self.marker_num = 1
        for (pose, occurances) in self.faces:
            if occurances >= self.occuranceThresh:
                self.marker_array.markers.append(self.make_marker(pose))
                self.marker_num += 1
        self.publisher.publish(self.marker_array)
        print("Markers updated")

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
    marker_org = marker_organizer(5, 0.25, False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker_org.check_faces()
        rate.sleep()


if __name__ == "__main__":
    main()
