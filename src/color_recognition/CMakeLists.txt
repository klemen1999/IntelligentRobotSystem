cmake_minimum_required(VERSION 3.0.2)
project(color_recognition)

find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    sensor_msgs
    dynamic_reconfigure
    message_generation
)

add_message_files(
  FILES
 )

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)


catkin_install_python(PROGRAMS
    scripts/randomForest.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )


