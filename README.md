# TeamEpsilon

# Ukazi za zagon usega:
```
roslaunch setup rins_run.launch
rosrun color_recognition randomForest.py
roslaunch cylinder_detection find_cylinder.launch 2>/dev/null
rosrun cylinder_detection cylinder_markers.py
rosrun ring_detection detect_rings 2>/dev/null
rosrun ring_detection ring_markers.py
rosrun move_arm move_arm.py
rosrun navigation move_robot.py
```
