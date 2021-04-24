cmake_minimum_required(VERSION 3.0.2)
project(ring_detection)

find_package(catkin REQUIRED
    roscpp
    std_msgs
    sensor_msgs
    dynamic_reconfigure
)


 
catkin_package()



catkin_install_python(PROGRAMS
    scripts/detect_rings
    scripts/ring_markers.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    
