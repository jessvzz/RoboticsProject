import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/americo/coding/ros2_ws/src/robot_description/install/robot_description'
