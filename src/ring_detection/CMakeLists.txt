cmake_minimum_required(VERSION 3.0.2)
project(ring_detection)

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
  RingPoseColor.msg
  RingDetection.msg
  RingsList.msg
 )

add_service_files(
  FILES
  RingVector.srv
 )
 
generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)
 
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)



catkin_install_python(PROGRAMS
    scripts/detect_rings
    scripts/ring_markers.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    
