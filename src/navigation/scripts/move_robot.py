#! /usr/bin/env python

import rospy
import os
import actionlib
import tf2_ros
import math
import time
from nav_msgs.msg import Odometry
from tf.transformations import *
from tf import LookupException, ConnectivityException
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3, Pose, Twist, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from face_detection.msg import ImageStatus
from std_msgs.msg import ColorRGBA, String
from sound.msg import RobotSpeakRequest
from navigation.msg import CalibrationMsg
from nav_msgs.srv import GetPlan
from auto_goals import AutoNav
from task import Task
from person import Person
from ring import Ring
from cylinder import Cylinder
from common_methods import color_name_from_rgba
from qr_and_number_detection.msg import DigitsMessage, QrMessage
from face_detection.srv import FaceNormal, FaceNormalRequest, FaceNormalResponse
from ring_detection.srv import RingVector, RingVectorRequest
from cylinder_detection.srv import CylinderStatus, CylinderStatusRequest


class move_controller():

    def __init__(self, points, debugStauts=False):
        if debugStauts:  # if we want to print status
            self.status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.print_status)

        self.points = points  # generated points
        # info about the robot position
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_odometry)
        self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        # Face stuff
        self.face_marker_sub = rospy.Subscriber('face_markers', MarkerArray, self.face_marker_received)
        rospy.wait_for_service("face_normal")
        self.face_normal_client = rospy.ServiceProxy("face_normal", FaceNormal)
        self.person_array = []
        self.visitedFaces = []
        # Ring stuff
        self.ring_marker_sub = rospy.Subscriber("ring_markers", MarkerArray, self.ring_marker_received)
        rospy.wait_for_service("ring_vector")
        self.ring_vector_client = rospy.ServiceProxy("ring_vector", RingVector)
        self.ring_array = []
        self.visitedRings = []
        # Cylinder stuff
        self.cylinder_sub = rospy.Subscriber('cylinder_markers', MarkerArray, self.cylinder_marker_received)
        rospy.wait_for_service("cylinder_status")
        self.cylinder_status_client = rospy.ServiceProxy("cylinder_status", CylinderStatus)
        self.cylinder_array = []
        self.visitedCylinders = []
        print("Got all of over services")
        # Digits stuff
        self.digits_sub = rospy.Subscriber('/digits', DigitsMessage, self.digits_callback)
        self.wait_for_digits = False
        self.current_person_age = 0
        # Qr code stuff
        self.qr_sub = rospy.Subscriber('/qr', QrMessage, self.qr_callback)
        self.wait_for_qr = False
        self.current_qr_data = None
        # Arm stuff
        self.arm_pub = rospy.Publisher("/arm_command", String, queue_size=2)
        # publisher for sound
        self.sound_pub = rospy.Publisher("robot_say", RobotSpeakRequest, queue_size=10)
        # move base client
        self.move_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move base server")
        self.move_client.wait_for_server()
        # markers to show next goal
        self.goal_publisher = rospy.Publisher('goal_markers', Marker, queue_size=1)
        self.marker_num = 10000
        # calibration status publisher
        self.calibration_pub = rospy.Publisher("calibration_status", CalibrationMsg, queue_size=10)
        # service to check if goal is reachable
        rospy.wait_for_service("/move_base/make_plan")
        self.goal_checker = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        self.visitedPoints = []
        self.distance_to_face = 0.6
        self.distance_to_ring = 0.45
        self.distance_to_cylinder = 0.45

    def print_status(self, data):
        if len(data.status_list) < 1:
            return
        output = "Status: " + str(data.status_list[-1].status) + " Text: " + str(data.status_list[-1].text)
        if data.status_list[-1].status == 3:
            rospy.loginfo(output)
        elif data.status_list[-1].status > 3:
            rospy.logwarn(output)

    def calibrate(self):
        # self.rotate(40, 360, True)
        msg = CalibrationMsg()
        msg.calibrationFinished = True
        self.calibration_pub.publish(msg)

    def get_odometry(self, msg):
        self.current_position = msg.pose.pose
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_rotation = yaw

    def main_loop(self):
        for point in self.points:
            pose, rotDeg, clockwise = point
            if not self.check_if_reachable(pose):
                continue
            if self.point_already_visited(pose):
                continue
            print("---\nMoving to next map goal")
            marker = self.make_marker(pose)
            self.goal_publisher.publish(marker)
            self.move(pose)
            print("Rotating around at the map goal")
            self.rotate(35, rotDeg, clockwise)


            if len(self.visitedFaces) < 3:
                # check for new face to approach
                print("Checking for faces to approach")
                self.check_face_approach()

            '''
            if len(self.visitedCylinders) < 4:
                print("Checking for cylinders to approach")
                self.check_cylinder_approach()

            
            if len(self.visitedRings) < 4:
                # check for new rings to approach
                print("Checking for rings to approach")
                self.check_ring_approach()
            
            if len(self.visitedRings) >= 4 and len(self.visitedCylinders) >= 4:
                print("Found everything. I'm gonna stop now.")
                return
            '''

        rospy.loginfo("End")

    def point_already_visited(self, given_pose):
        for pose in self.visitedPoints:
            dist = self.euclid_distance(pose.position, given_pose.position)
            if dist < 1:
                return True
        return False

    def euclid_distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

    def move(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose = pose
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()
        self.visitedPoints.append(pose)

    def rotate(self, speed, angle, clockwise):
        speed_rad = self.deg_to_radian(speed)
        angular_speed = self.get_angular_speed(speed_rad, clockwise)
        angle = self.deg_to_radian(angle)
        threshold = 0.05
        rate = rospy.Rate(100)

        vel_msg = self.init_vel_msg()

        current_rotation = self.current_rotation
        rotated = 0

        while not self.close_to(rotated, angle, threshold):
            vel_msg.angular.z = angular_speed
            self.velocity_pub.publish(vel_msg)

            prev_rotation = current_rotation
            current_rotation = self.current_rotation

            rotated += abs(abs(current_rotation) - abs(prev_rotation))
            rate.sleep()

        vel_msg.angular.z = 0
        self.velocity_pub.publish(vel_msg)

    def close_to(self, value, reference, threshold):
        return ((reference + threshold) >= value >= reference) \
               or ((reference - threshold) <= value <= reference)

    def get_angular_speed(self, angular_speed, clockwise):
        if (clockwise):
            return -abs(angular_speed)
        else:
            return abs(angular_speed)

    def deg_to_radian(self, deg):
        return deg * 2 * math.pi / 360

    def init_vel_msg(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        return vel_msg

    def face_marker_received(self, msg):
        self.face_marker_array = msg

        if len(self.face_marker_array.markers) > len(self.person_array):
            new_face_marker = self.face_marker_array.markers[-1]
            new_person = Person(new_face_marker.id, new_face_marker.pose)
            self.person_array.append(new_person)
            print(f"new person added: {new_person}")
        else:
            for i in range(len(self.person_array)):
                current_marker = self.face_marker_array.markers[i]
                current_person = self.person_array[i]
                current_person.update_from_marker(current_marker)

    def check_face_approach(self):
        try:
            if self.face_marker_array != None and self.face_marker_array.markers != None and \
                    len(self.face_marker_array.markers) > 0:
                self.move_to_faces()
        except Exception as e:
            print("No faces detected yet")
            print(f"Possible error: {e}")

    def move_to_faces(self):
        for marker in self.face_marker_array.markers:
            # check if you already visited the marker
            if marker.id in self.visitedFaces:
                continue
            # requesting normal vec of current face marker
            request = FaceNormalRequest()
            request.markerID = marker.id
            response = self.face_normal_client(request)
            # don't approach markers with not enough occurances
            if not response.viable:
                continue
            normalVec = [response.unitNormal[0], response.unitNormal[1]]
            # calculate pose for approach
            pose = self.approach_transform(marker.pose, normalVec, self.distance_to_face)
            if self.check_if_reachable(pose):
                # adding marker to see next approach
                markerToFace = self.make_marker(pose)
                self.goal_publisher.publish(markerToFace)
                self.wait_for_digits = True
                self.wait_for_qr = True
                print("Moving to approach the face")
                self.move(pose)
                print("Saying hello to face")
                self.speak("Hello face")

                rotated_for_digits = False
                # Rotate towards digits

                if (self.wait_for_digits):
                    self.rotate(10, 20, False)
                    rotated_for_digits = True

                timeout = time.time() + 5

                while (self.wait_for_digits and time.time() < timeout):
                    print("Waiting for digits")
                    rospy.sleep(0.5)

                # Rotate towards qr code
                if (self.wait_for_qr):
                    print("Rotating towards qr code")
                    if (rotated_for_digits):
                        self.rotate(10, 40, True)
                    else:
                        self.rotate(10, 20, True)

                print(f"Current person age is: {self.current_person_age}")
                print(f"Current qr data is: {self.current_qr_data}")

                self.visitedFaces.append(marker.id)  # add marker id you already visited
            else:
                print("Can't reach the face")

    def ring_marker_received(self, msg):
        self.ring_marker_array = msg

        if len(self.ring_marker_array.markers) != len(self.ring_array):
            new_ring_marker = self.ring_marker_array.markers[-1]
            new_ring = Ring(new_ring_marker.id,
                            new_ring_marker.pose,
                            color_name_from_rgba(new_ring_marker.color))

            self.ring_array.append(new_ring)
            print(f"New ring found:\n {new_ring}")
        else:
            for i in range(len(self.cylinder_array)):
                current_ring = self.cylinder_array[i]
                current_marker = self.ring_marker_array.markers[i]
                current_ring.update_from_marker(current_marker)

    def digits_callback(self, msg):
        if (self.wait_for_digits):
            print(f"Digits received: first --> {msg.first_digit} , second --> {msg.second_digit} ")
            self.current_person_age = msg.first_digit * 10 + msg.second_digit
            self.wait_for_digits = False

    def qr_callback(self, msg):
        if (self.wait_for_qr):
            print(f"Qr data received: {msg.qr_data}")
            self.current_qr_data = msg.qr_data
            self.wait_for_qr = False

    def check_ring_approach(self):
        try:
            if self.ring_marker_array != None and self.ring_marker_array.markers != None and \
                    len(self.ring_marker_array.markers) > 0:
                self.move_to_rings()
        except Exception as e:
            print(f"Exception {e}")
            print("No rings detected yet")

    def move_to_rings(self):
        for marker in self.ring_marker_array.markers:
            # check if you already visited the marker
            if marker.id in self.visitedRings:
                continue
            # requesting vector of current ring marker
            request = RingVectorRequest()
            request.markerID = marker.id
            response = self.ring_vector_client(request)
            # don't approach markers with not enough occurances
            if not response.viable:
                continue
            color = response.color
            vector = [response.unitVector[0], response.unitVector[1]]
            # calculate pose for approach (try original and negative vector)
            pose1 = self.approach_transform(marker.pose, vector, self.distance_to_ring)
            vectorNegative = [-x for x in vector]
            pose2 = self.approach_transform(marker.pose, vectorNegative, self.distance_to_ring)
            if self.check_if_reachable(pose1):
                pose = pose1
            elif self.check_if_reachable(pose2):
                pose = pose2
            else:
                print("Can't reach the ring")
                continue
            # adding marker to see next approach
            markerToFace = self.make_marker(pose)
            self.goal_publisher.publish(markerToFace)
            print("Moving to approach the", color, "ring")
            self.move(pose)
            distance = self.distance_to_ring + 0.1
            self.close_approach(distance, True)
            print("Now under the ring")
            print(f"Saying hello to {color} ring")
            self.speak(f"Hello {color} ring")
            self.close_approach(distance, False)
            self.visitedRings.append(marker.id)  # add marker id you already visited

    def cylinder_marker_received(self, msg):
        self.cylinder_markers = msg

        if len(self.cylinder_markers.markers) != len(self.cylinder_array):
            new_cylinder_marker = self.cylinder_markers.markers[-1]
            new_cylinder = Cylinder(new_cylinder_marker.id,
                                    new_cylinder_marker.pose,
                                    color_name_from_rgba(new_cylinder_marker.color),
                                    self.current_position)

            self.cylinder_array.append(new_cylinder)
            print(f"New cylinder found:\n {new_cylinder}")
        else:
            for i in range(len(self.cylinder_array)):
                current_cylinder = self.cylinder_array[i]
                current_marker = self.cylinder_markers.markers[i]
                current_cylinder.update_from_marker(current_marker)

    def check_cylinder_approach(self):
        try:
            if self.cylinder_markers != None and self.cylinder_markers.markers != None and \
                    len(self.cylinder_markers.markers) > 0:
                self.move_to_cylinders()
        except Exception as e:
            print("No cylinders detected yet")
            print(f"Possible error: {e}")

    def move_to_cylinders(self):
        for marker in self.cylinder_markers.markers:
            # check if you already visited the marker
            if marker.id in self.visitedCylinders:
                continue
            # calculating pose for approach
            request = CylinderStatusRequest()
            request.markerID = marker.id
            response = self.cylinder_status_client(request)
            color = response.color
            # don't approach markers with not enough occurances
            if not response.viable:
                continue
            angleAdd = self.deg_to_radian(20)
            pose = self.approach_transform_original(self.current_position, marker.pose, self.distance_to_cylinder, angleAdd)
            if self.check_if_reachable(pose):
                # adding marker to see next approach
                markerToFace = self.make_marker(pose)
                self.wait_for_qr = True
                self.goal_publisher.publish(markerToFace)
                print("Moving to approach the", color, "cylinder")
                self.move(pose)
                distance = self.distance_to_cylinder
                # self.close_approach(distance, True)
                print(f"Saying hello to {color} cylinder and extending arm")
                self.move_arm("extend")
                self.speak(f"Hello {color} cylinder")
                self.move_arm("retract")
                # self.close_approach(distance, False)

                marker_relative_to_me = self.marker_relative_to_robot(pose)
                where_am_i_rotated = self.where_am_i_rotated()
                print(f"Marker relative: {marker_relative_to_me} \n my_relative_rotation {where_am_i_rotated}")

                if self.wait_for_qr:
                    print("Didn't find qr code on cylinder, trying to move around it")
                    self.move_around_cylinder(marker.pose)

                self.visitedCylinders.append(marker.id)  # add marker id you already visited
            else:
                print("Can't reach the cylinder")

    def move_arm(self, action):
        if action == "retract" or action == "extend":
            msg = String()
            msg.data = action
            self.arm_pub.publish(msg)
            rospy.sleep(2)
        else:
            print("Unknown arm command")

    def approach_transform(self, markerPose, vector, scale):
        pose = Pose()
        vector = [x * scale for x in vector]  # multiply the normal vector to get right distance to face
        pose.position = Vector3(markerPose.position.x + vector[0], markerPose.position.y + vector[1], 0)
        pose.orientation = self.look_at(pose, markerPose)
        return pose

    def approach_transform_original(self, curr_pose, target_pose, scale, angleAdd):
        dx = target_pose.position.x - curr_pose.position.x
        dy = target_pose.position.y - curr_pose.position.y
        v = Vector3(dx, dy, 0)
        v_len = math.sqrt(math.pow(v.x, 2) + math.pow(v.y, 2))
        v_new_len = v_len - scale
        v_mul = v_new_len / v_len
        v = Vector3(v.x * v_mul, v.y * v_mul, 0)
        v = Vector3(v.x + curr_pose.position.x, v.y + curr_pose.position.y, 0)
        rad = math.atan2(dy, dx) + angleAdd
        q = quaternion_from_euler(0, 0, rad)
        q = Quaternion(q[0], q[1], q[2], q[3])
        pose = Pose(v, q)
        return pose

    def move_forward(self, distance, speed):
        starting_position = self.current_position.position
        twist = Twist()
        threshold = 0.5

        twist.linear.x = speed

        current_moved_distance = self.euclid_distance(self.current_position.position, starting_position)

        while (not self.close_to(current_moved_distance, distance, threshold)):
            self.velocity_pub.publish(twist)
            rospy.sleep(0.5)

        twist.linear.x = 0
        self.velocity_pub.publish(twist)

    def close_approach(self, distance, forward):
        vel_msg = self.init_vel_msg()
        if forward:
            vel_msg.linear.x = distance
        else:
            vel_msg.linear.x = -distance
        self.velocity_pub.publish(vel_msg)
        rospy.sleep(1)

    def check_if_reachable(self, targetPose):
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose = self.current_position
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose = targetPose
        tolerance = 0
        response = self.goal_checker(start, goal, tolerance)
        if len(response.plan.poses) > 0:
            return True
        else:
            return False

    def look_at(self, start_pose, end_pose):
        dx = end_pose.position.x - start_pose.position.x
        dy = end_pose.position.y - start_pose.position.y
        rad = math.atan2(dy, dx)
        q = quaternion_from_euler(0, 0, rad)
        q = Quaternion(q[0], q[1], q[2], q[3])
        return q

    def speak(self, message):
        soundMsg = RobotSpeakRequest()
        soundMsg.message = message
        self.sound_pub.publish(soundMsg)

    def make_marker(self, pose):
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose = pose
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(30)
        marker.id = self.marker_num
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0.73, 0.25, 0.35, 1)
        self.marker_num += 1
        return marker

    def move_around_cylinder(self, cylinder_position):
        print(f"Current rotation: {self.current_rotation} \n "
              f"my_position: {self.current_position} \n "
              f"Marker position: {cylinder_position}")

        marker_relative_to_me = self.marker_relative_to_robot(cylinder_position)
        where_am_i_rotated = self.where_am_i_rotated()
        print(f"Marker relative: {marker_relative_to_me} \n my_relative_rotation {where_am_i_rotated}")
        new_point = self.calculate_new_point(cylinder_position.position, marker_relative_to_me, where_am_i_rotated, 0.6)
        print(f"New point\n{new_point}")
        self.move(new_point)

    def marker_relative_to_robot(self, marker_position):
        marker = marker_position.position
        me = self.current_position.position
        delta_x = me.x - marker.x
        delta_y = me.y - marker.y

        if abs(delta_x) > abs(delta_y):
            if me.x < marker.x:
                return "UP"
            else:
                return "DOWN"
        else:
            if me.y < marker.y:
                return "LEFT"
            else:
                return "RIGHT"

    def where_am_i_rotated(self):
        my_rotation = self.current_rotation
        if my_rotation > 0:
            if my_rotation > (math.pi / 2):
                return "LD"
            else:
                return "LU"
        else:
            if my_rotation < (-math.pi / 2):
                return "RD"
            else:
                return "RU"

    def calculate_new_point(self, marker_position, relative_position, orientation, distance):
        pose = Pose()
        pose.position.z = 0
        quaternion = []
        if relative_position == "LEFT" or relative_position == "RIGHT":
            pose.position.y = marker_position.y
            if "D" in orientation:
                pose.position.x = marker_position.x + distance
                quaternion = quaternion_from_euler(0, 0, math.pi)
            else:
                pose.position.x = marker_position.x - distance
                quaternion = quaternion_from_euler(0, 0, 0)
        else:
            pose.orientation.x = marker_position.x
            if "L" in orientation:
                pose.position.y = marker_position.y + distance
                quaternion = quaternion_from_euler(0, 0, (math.pi / 2))
            else:
                pose.position.y = marker_position.y - distance
                quaternion = quaternion_from_euler(0, 0, (-math.pi / 2))

        self.set_orientation_from_quaternion_array(pose, quaternion)
        return pose

    def set_orientation_from_quaternion_array(self, pose, quaternion):
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

    def ghetto_move(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = -0.17239
        goal.target_pose.pose.position.y = -0.8683148
        goal.target_pose.pose.orientation.z = -0.63146085
        goal.target_pose.pose.orientation.w = 0.7754
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()


def main():
    rospy.init_node("move_robot_node")
    print("Getting autoNavigator goals")
    autoNavigator = AutoNav(33, True)
    points = autoNavigator.get_mapGoals()
    mover = move_controller(points, False)
    print("Calibrating")
    mover.calibrate()
    print("Going to goals")
    mover.main_loop()


if __name__ == "__main__":
    main()
