cmake_minimum_required(VERSION 2.8.3)
project(face_detection)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	visualization_msgs
	tf2_ros
	nav_msgs
	std_msgs
	geometry_msgs
	message_generation
)

find_package( OpenCV REQUIRED )

add_message_files(
  FILES
  ImageStatus.msg
  FacePoses.msg
 )

add_service_files(
  FILES
  FaceNormal.srv
 )

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
 )

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs
 )

#include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#add_executable(map_goals src/map_goals.cpp)
#target_link_libraries(map_goals ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#catkin_install_python(PROGRAMS scripts/detect_rings DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS
    scripts/face_localizer_dlib 
    scripts/face_localizer_dnn 
    scripts/face_markersOld.py
    scripts/face_markers.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )


