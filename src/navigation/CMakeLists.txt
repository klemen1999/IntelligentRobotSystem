cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	cv_bridge
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf2
	tf2_geometry_msgs
)

find_package(OpenCV)

catkin_python_setup()

add_message_files(
  FILES
  CalibrationMsg.msg
 )
 
generate_messages(
	DEPENDENCIES
	std_msgs
 )


catkin_package(CATKIN_DEPENDS message_runtime roscpp rospy std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
    scripts/move_robot.py scripts/auto_goals.py scripts/move_robotOld.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



