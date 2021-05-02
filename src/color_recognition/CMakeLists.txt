cmake_minimum_required(VERSION 3.0.2)
project(color_recognition)

find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    sensor_msgs
    dynamic_reconfigure
    message_generation
    geometry_msgs
)

add_message_files(
  FILES
  Int2dArray.msg
  IntList.msg
 )

add_service_files(
  FILES
  ObjectColor.srv
 )

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)


catkin_install_python(PROGRAMS
    scripts/randomForest.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )


