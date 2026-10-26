# RoboticsProject summary

This workspace is a ROS 2 + Gazebo project for a warehouse robot that should detect physical package markers (AprilTags), navigate to them, pick them up, and then drive to a designated dropoff zone marked by another AprilTag.

In other words, the project is structured around the following loop:

1. Robot sees AprilTags with its camera.
2. The tag detector estimates the tag position in camera coordinates.
3. The mapper converts that into the robot/world frame and caches tag locations by ID.
4. Higher-level task logic (not yet fully implemented here) can decide: go to package tag -> grab -> go to dropoff tag -> release.

## Workspace layout

### `src/april_tag_detection/`
This is the main perception package.

- `april_tag_detection/april_tag_detector.py`
  - ROS 2 node that subscribes to the TurtleBot camera image and camera info.
  - Uses `pupil_apriltags` to detect AprilTags.
  - Estimates each tag's 3D pose relative to the camera.
  - Publishes `geometry_msgs/PointStamped` on `/apriltag_detection/tag_detection`.
  - Publishes a debug image on `/apriltag_detection/debug_image`.

- `april_tag_detection/april_tag_mapper.py`
  - Subscribes to tag detections.
  - Looks up the TF transform from the camera frame to the world frame (`odom` is used here).
  - Stores the tag's global position in a dictionary keyed by tag ID.
  - Exposes a service `GetTagPosition` so other nodes can request the map/world position of a tag.
  - Publishes visualization markers for RViz.

- `launch/start_detection.launch.py`
  - Starts the detector and mapper without visualization tools.

- `launch/start_detection_debug.launch.py`
  - Starts the detector + mapper + RViz + rqt image view for debugging.

- `rviz/`
  - Prebuilt RViz configs for visualizing the camera feed and tag markers.

### `src/custom_interfaces/`
Defines custom ROS interfaces, especially:

- `srv/GetTagPosition.srv`
  - Request: `tag_id`
  - Response: `found`, `position`

This is the interface other higher-level robot logic would use when asking “where is tag X in the world?”

### `src/forklift_manager/`
This package is intended to handle the actual forklift or manipulator behavior.

- It is designed as the interface for actions like:
  - `grab_package`
  - `release_package`
  - `move_forklift`
- The README says this should expose service-based actions for forklift hardware or simulation.
- Right now, it is mostly a scaffold and not the full pickup/dropoff logic.

### `src/turtlebot4_simulator/`
This contains the Gazebo/TurtleBot simulation stack.

- `turtlebot4_gz_bringup/launch/turtlebot4_gz.launch.py`
  - Main launch file for starting the robot in simulation.
- `turtlebot4_gz_bringup/launch/sim.launch.py`
  - Launches Gazebo Harmonic and sets important resource paths.
- `turtlebot4_gz_bringup/worlds/`
  - Includes world files such as `maze.sdf`, `warehouse.sdf`, and others.
- `turtlebot4_gz_bringup/worlds/maze.sdf.xacro`
  - Xacro world that builds a maze-like environment and includes AprilTag models.

This is the simulation layer the AprilTag detector is intended to work with.

### `src/assets/models/`
Contains Gazebo models for AprilTags.

- Example: `Apriltag36_11_00013/`, `Apriltag36_11_00012/`, etc.
- These are required so Gazebo can spawn tags in the world.
- This is exactly the kind of asset needed for package and dropoff markers in the warehouse.

### `src/requirements.txt`
Lists Python dependencies used by the project, including `apriltag` and `pupil-apriltags`.

## What is implemented vs. missing

What is present:

- Camera-based AprilTag detection in simulation
- Tag localization relative to the camera
- Tag-to-world TF conversion
- Tag cache lookup service
- Gazebo world + TurtleBot robot setup
- AprilTag model library for Gazebo

What is still missing or incomplete:

- Full navigation behavior to reach package and dropoff poses
- Real “grab package” / “release package” logic
- Higher-level mission/state machine for package pickup and dropoff
- More complete robot controller integration with the forklift mechanism

So the repo is mostly a solid perception + simulation foundation for the bigger warehouse robot task, not a finished autonomous warehouse system.

## How to run the project

From the workspace root:

```bash
cd /home/americo/coding/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install
```

Then launch the Gazebo simulation:

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=maze
```

Then launch the tag detection stack:

```bash
ros2 launch april_tag_detection start_detection_debug.launch.py
```

or:

```bash
ros2 launch april_tag_detection start_detection.launch.py
```

The debug version starts RViz and rqt image view, which is the easiest way to see the camera feed and AprilTag detections.

## Running Gazebo with a custom map

The world parameter is the key part.

In `turtlebot4_gz_bringup/launch/turtlebot4_gz.launch.py`, the launch file accepts a `world` argument and passes it into the Gazebo simulation. This means you can launch a map by its world name, as long as the `.sdf` file exists in the world folder.

Example:

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=maze
```

This uses the file:

```text
src/turtlebot4_simulator/turtlebot4_gz_bringup/worlds/maze.sdf
```

To use your own custom world:

1. Create or place a `.sdf` file in:
   ```text
   src/turtlebot4_simulator/turtlebot4_gz_bringup/worlds/
   ```
2. Make sure your AprilTag models are in the Gazebo resources path, usually via `assets/models`.
3. Launch with the file name without the `.sdf` extension:

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=my_custom_map
```

If your map is not in that folder, you can also add its directory to `GZ_SIM_RESOURCE_PATH` before launching Gazebo.

Example:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/americo/coding/ros2_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/worlds:/home/americo/coding/ros2_ws/src/assets/models
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=my_custom_map
```

That is the cleanest way to plug in a custom Gazebo map while keeping the robot simulation and AprilTag models available.

## Bottom line

This is a ROS 2 robot-perception project built around AprilTags, Gazebo simulation, and TurtleBot 4. It already has the tag-detection pipeline working and the world-model infrastructure in place. The missing piece is the task-level logic for package pickup and dropoff behavior, which is where the forklift manager and higher-level robot controller will eventually plug in.
