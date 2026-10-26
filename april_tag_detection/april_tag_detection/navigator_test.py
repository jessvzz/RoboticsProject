import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions

rclpy.init()
navigator = TurtleBot4Navigator()

# Wait for Nav2 to initialize its costmaps and localization
navigator.waitUntilNav2Active()

# Undock to allow movement (essential even in simulation to clear the 0,0 constraint)
navigator.undock()

# Create a PoseStamped goal [X, Y] and facing direction
goal_pose = navigator.getPoseStamped([1.0, 0.0], TurtleBot4Directions.NORTH)

# Dispatch the goal to the Nav2 action server
navigator.startToPose(goal_pose)