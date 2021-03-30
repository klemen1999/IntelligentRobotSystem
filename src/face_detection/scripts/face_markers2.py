#!/usr/bin/python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose
from std_msgs.msg import ColorRGBA
from face_detection.msg import ImageStatus
from nav_msgs.msg import Odometry

class marker_organizer():

    def __init__(self, occuranceThresh, distThresh):
        rospy.init_node('face_markers_node')
        self.subscriber = rospy.Subscriber("face_pose", Pose, self.new_detection)
        self.buffer = []  # buffer to catch poses from face_pose topic
        self.publisher = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)
        self.img_status_pub = rospy.Publisher('face_status', ImageStatus, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_robot_position)
        self.faces = []
        self.marker_array = MarkerArray()
        self.marker_num = 1
        self.occuranceThresh = occuranceThresh
        self.distThresh = distThresh
        self.robot_position = []

    def new_detection(self, pose):
        print("got pose")
        self.buffer.append(pose)

    def img_in_the_middle(self, x, y):
        first_x1 = 0.2
        first_x2 = 0.6
        first_y1 = -0.6
        first_y2 = 1.6

        second_x1 = 0.4
        second_x2 = 1.6
        second_y1 = -0.65
        second_y2 = 0

        if first_x1 <= x <= first_x2 and first_y1 <= y <= first_y2:
            #print("img at the first middle wall!")
            return 0    #img is on the first middle wall
        elif second_x1 <= x <= second_x2 and second_y1 <= y <= second_y2:
            #print("img at the second middle wall!")
            return 1    #img is on the second mid wall
        return -1   #img is not in the middle

    def get_robot_position(self, msg):  #get robots x,y position
        p = msg.pose.pose.position
        x = p.x
        y = p.y
        self.robot_position = [x,y]
    
    def check_side(self, wall, x_robot, y_robot):
        if wall == 0:   #first middle wall
            if x_robot > 0.4:   #check on which side it is
                return 0
            else:
                return 1
        elif wall == 1: #second mid wall
            if y_robot < -0.4:  #check on which side the robot is
                return 0
            else:
                return 1
        return -1   #img is not in the middle walls

    def check_faces(self):

        for pose in self.buffer:
            noMatch = 0

            check_midle = self.img_in_the_middle(pose.position.x, pose.position.y)
            
            check_side = self.check_side(check_midle, self.robot_position[0], self.robot_position[1])

            for i, (face, occurances, mid, side) in enumerate(self.faces):
                numMatches = 0
                if face.position.x - self.distThresh <= pose.position.x \
                        <= face.position.x + self.distThresh:
                    numMatches += 1
                if face.position.y - self.distThresh <= pose.position.y \
                        <= face.position.y + self.distThresh:
                    numMatches += 1
                # zna bit problem če so dve sliki na isti steni ampak na drugi strani
                
                if numMatches == 2:
                    #face matches x,y
                    if check_midle > -1:
                        #face is at the middle walls
                        if check_side == side:  #face at mid, same side

                            face.position.x = (face.position.x * occurances
                                            + pose.position.x) / (occurances + 1)
                            face.position.y = (face.position.y * occurances
                                            + pose.position.y) / (occurances + 1)
                            occurances += 1
                            self.faces[i] = (face, occurances, mid, side)
                            print("middle, same side")
                            if occurances == self.occuranceThresh:
                                if self.check_clusters(i):
                                    self.update_markers()
                        else:   #face at mid, other side
                            noMatch += 1
                            print("middle, but other side, add new face")
                    else: #img not in the middle, so no doubt
                        face.position.x = (face.position.x * occurances
                                            + pose.position.x) / (occurances + 1)
                        face.position.y = (face.position.y * occurances
                                            + pose.position.y) / (occurances + 1)
                        occurances += 1
                        self.faces[i] = (face, occurances, mid, side)
                        if occurances == self.occuranceThresh:
                            if self.check_clusters(i):
                                self.update_markers()
                else:   #no match x,y -> new face
                    noMatch += 1
                    print("no match")

            if noMatch == len(self.faces):
                status_message = ImageStatus()
                status_message.status = "NEW_FACE"
                self.img_status_pub.publish(status_message)
                self.faces.append((pose, 1, check_midle, check_side))
        if len(self.faces) > 5:
            rospy.logwarn("NOVO!!!")
            for a in self.faces:
                print(a[0], a[1])
        self.buffer = []

    def check_clusters(self, ix):
        (pose, occ, mi, si) = self.faces[ix]
        for i, (face, occurances, mid, side) in enumerate(self.faces):
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
                self.faces[i] = (face, occurances, mid, side)
                return False
        return True

    def update_markers(self):
        self.marker_array.markers = []
        self.marker_num = 1
        for (pose, occurances, mid, side) in self.faces:
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
    #marker_org = marker_organizer(5, 0.5)
    #TODO switch back
    marker_org = marker_organizer(5, 1.6)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker_org.check_faces()
        rate.sleep()


if __name__ == "__main__":
    main()
