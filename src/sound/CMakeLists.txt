cmake_minimum_required(VERSION 2.8.3)
project(sound)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	sensor_msgs
	sound_play
	std_msgs
)

find_package(OpenCV)

add_message_files(
  FILES
  RobotSpeakRequest.msg
 )

generate_messages(
	DEPENDENCIES
	std_msgs
 )

catkin_package(CATKIN_DEPENDS message_runtime roscpp rospy std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
    scripts/robot_sound.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



