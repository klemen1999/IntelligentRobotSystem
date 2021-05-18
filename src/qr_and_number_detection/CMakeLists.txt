cmake_minimum_required(VERSION 2.8.3)
project(qr_and_number_detection)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	visualization_msgs
	tf2_ros
	nav_msgs
	std_msgs
	geometry_msgs
)

find_package( OpenCV REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

add_message_files(DIRECTORY msg
  FILES
  DigitsMessage.msg
  QrMessage.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package()

#include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#add_executable(map_goals src/map_goals.cpp)
#target_link_libraries(map_goals ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

catkin_install_python(PROGRAMS scripts/extract_digits.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/extract_qr.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

