# Perception-Driven Pick & Place in Ignition Gazebo using ROS 2 & MoveIt Task Constructor

## Overview

This project implements a perception-to-motion pipeline for the academy robot in Ignition Gazebo using ROS 2 Humble and MoveIt Task Constructor (MTC).

The final working system does the following:
1. reads the RGBD point cloud from the simulated camera,
2. isolates the screw object,
3. estimates its pose by registering the observed cloud to the provided STL model,
4. publishes and serves the detected pose,
5. sends a `PickPlace` action goal,
6. plans and executes a simplified MTC task in Gazebo.

The final manipulation pipeline is a stable simplified version:
- current state
- move arm to `pregrasp`
- open hand
- move arm to `place_pregrasp`
- open hand

This gives a reliable end-to-end demo, even though it is not a full grasp-lift-transport-place pipeline.

---

## Repository Structure

### `academy_robot_pick_place`
- `src/object_detector_node.cpp`
- `src/pick_place_server.cpp`
- `src/pick_place_client_node.cpp`
- `launch/pick_place.launch.py`
- `CMakeLists.txt`
- `package.xml`
- `README.md`

### `academy_robot_interfaces`
- `srv/DetectObject.srv`
- `action/PickPlace.action`

---

## ROS 2 Interfaces

### Inputs
- `/front_rgbd_camera/depth/color/points`
- `/joint_states`

### Outputs
- `/detected_object_pose` → `geometry_msgs/msg/PoseStamped`
- `/detect_object` → `academy_robot_interfaces/srv/DetectObject`
- `/pick_place_task` → `academy_robot_interfaces/action/PickPlace`

### `DetectObject.srv`
---
bool success
geometry_msgs/PoseStamped detected_pose

### `PickPlace.action`
geometry_msgs/PoseStamped target_pose
geometry_msgs/PoseStamped place_pose
string object_id
---
bool success
int64 planning_time_ms
int64 execution_time_ms
---
string current_stage

---

## System Architecture

The system is split into three layers:

### 1) Simulation layer
Ignition Gazebo provides:
- the world,
- the screw object,
- the academy robot,
- the RGBD point cloud,
- and the robot controllers.

### 2) Perception layer
`object_detector_node`:
- subscribes to `/front_rgbd_camera/depth/color/points`,
- isolates the screw using geometry-based seed clustering,
- estimates a 6D pose using PCL ICP against the provided STL,
- transforms the result into `robot_odom`,
- publishes `/detected_object_pose`,
- serves `/detect_object`.

### 3) Manipulation layer
`pick_place_client_node`:
- calls `/detect_object`,
- receives the detected pose,
- builds a `PickPlace` action goal,
- sends it to `/pick_place_task`.

`pick_place_server`:
- receives the action goal,
- builds the MTC task,
- streams stage feedback,
- plans and executes through MoveIt / `move_group`,
- drives Gazebo through the arm and gripper trajectory controllers.

### Data flow
Ignition Gazebo point cloud  
→ `object_detector_node`  
→ `/detected_object_pose` + `/detect_object`  
→ `pick_place_client_node`  
→ `/pick_place_task`  
→ `pick_place_server`  
→ MoveIt / `move_group` / MTC  
→ Gazebo controllers  
→ robot motion in simulation

---

## Implementation Summary

## T1–T4: Object detection
The detector uses the organized point cloud directly instead of reconstructing 3D data from separate RGB and depth streams.

### Final T2 method
The working T2 method is geometry-based:
- restrict search to a camera ROI,
- find the nearest valid seed point,
- grow a local cluster around that seed,
- keep the isolated object cloud.

This replaced the earlier color-threshold approach, which was unreliable for the simulated point cloud.

### Final T3 method
The working T3 method uses:
- STL loading,
- STL-to-point-cloud conversion,
- downsampling,
- PCL ICP registration,
- pose extraction,
- TF transform to `robot_odom`.

Important working values:
- `max_depth_m=10.0`
- `reference_scale=0.001`
- `voxel_leaf_size_m=0.0015`
- `icp_max_corr_m=0.03`
- `icp_max_iterations=80`
- `icp_fitness_threshold=0.02`

### Final T4 behavior
The node:
- continuously publishes the latest pose on `/detected_object_pose`,
- caches the latest valid pose,
- returns that cached pose through `/detect_object`.

---

## T5–T6: Custom interfaces
The project defines:
- `DetectObject.srv`
- `PickPlace.action`

Both are implemented in `academy_robot_interfaces` and are used successfully by the detector, client, and server.

---

## T7–T10: MTC action server
The final server is implemented in `src/pick_place_server.cpp`.

After several debugging iterations, the stable pipeline was simplified to:
- current state
- move arm to `pregrasp`
- open hand
- move arm to `place_pregrasp`
- open hand

The server:
- initializes the task,
- plans through MTC,
- executes through MoveIt,
- publishes feedback stages,
- returns planning and execution timing.

---

## T11–T13: Client node
The client:
- calls `/detect_object`,
- receives a pose in `robot_odom`,
- builds a `PickPlace` goal,
- sends it to `/pick_place_task`,
- prints feedback and final result.

The final end-to-end path works when the system is launched correctly and a sufficiently large timeout is used.

---

## Key Design Choices

### Perception
- direct use of `PointCloud2`
- geometry-based isolation instead of color thresholding
- PCL ICP instead of Open3D
- cached latest pose for the detection service

### Manipulation
- simplified named-state MTC pipeline
- priority on a reliable full demo path over a more ambitious unstable pipeline

---

## Current Status

### Working
- object isolation
- ICP-based pose estimation
- pose publishing and detection service
- custom service and action interfaces
- action client/server communication
- MTC planning and Gazebo execution for the final simplified pipeline

### Current limitation
The final MTC task is a simplified stable sequence, not a full grasp / lift / transport / place implementation.

---

## Opening Docker

### Open the running container as normal user
`docker exec -it academy_robotics /bin/bash`

### Open the running container as root
`docker exec -u 0 -it academy_robotics /bin/bash`

---

## Installing the required MoveIt Task Constructor packages

Open the container as root, then run:

`source /opt/ros/humble/setup.bash`

`apt update`

`apt install -y ros-humble-moveit-task-constructor-core ros-humble-moveit-task-constructor-msgs ros-humble-moveit-task-constructor-capabilities ros-humble-moveit-task-constructor-visualization`

---

## Build

Inside Docker:

`cd /home/user/ros2_ws`

`source /opt/ros/humble/setup.bash`

`colcon build --packages-select academy_robot_interfaces academy_robot_moveit_config academy_robot_pick_place`

`source install/setup.bash`

---

## One-file launch

Inside Docker:

`cd /home/user/ros2_ws`

`source /opt/ros/humble/setup.bash`

`source install/setup.bash`

`ros2 launch /home/user/ros2_ws/src/academy_robot_pick_place/launch/pick_place.launch.py result_timeout_sec:=900.0`

---

## Manual 5-terminal launch

In each terminal:

`docker exec -it academy_robotics /bin/bash`

`cd /home/user/ros2_ws`

`source /opt/ros/humble/setup.bash`

`source install/setup.bash`

### Terminal 1
`ros2 launch academy_robot_gazebo_ignition spawn_world.launch.py spawn_screw:=true`

### Terminal 2
`ros2 launch academy_robot_gazebo_ignition spawn_robot.launch.py robot:=academy_robot robot_model:=academy_robot_plus has_arm:=true`

### Terminal 3
`ros2 run academy_robot_pick_place object_detector_node`

### Terminal 4
`ros2 run academy_robot_pick_place pick_place_server`

### Terminal 5
`ros2 run academy_robot_pick_place pick_place_client_node --ros-args -p result_timeout_sec:=420.0`

---

## Notes for Running

- `spawn_screw:=true` is required so the screw appears in the world.
- The detector publishes in `robot_odom`.
- Execution can take several minutes, so the client timeout must be large enough.
- The 5-terminal launch remains the safest demo path if the one-file launch is unstable in a given session.

---

## Final Summary

The repository demonstrates a working perception-to-execution path in ROS 2 Humble and Ignition Gazebo.

Strong points:
- geometry-based object isolation,
- registration-based pose estimation using the provided STL,
- clean ROS 2 topic / service / action separation,
- successful end-to-end execution with feedback and timing.

Main limitation:
- the final MTC pipeline is a simplified stable version rather than a complete pick-and-place manipulation pipeline.
