HOW TO LAUNCH EVERYTHING:
-
To start this system, we first need to have a gazebo simulation running. After sourcing ros2 and the project workspace, run
this command to launch the gazebo simulator with turtlebot robot:

> ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=maze  


(For more in-depth info about turtlebot4, visit https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html)

After the gazebo simulation is running, you can launch the april tag detection system by running one of these two commands:

1) > ros2 launch april_tag_detection start_detection_debug.launch.py
2) > ros2 launch april_tag_detection start_detection.launch.py

Choose opt.1 if you want to launch the core apriltag detection system as well as visualization tools (rviz, rqt);  
 choose opt.2 to just launch the core apriltag detection system (don't start visualization tools).