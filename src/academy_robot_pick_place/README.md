# Perception-Driven Pick & Place in Ignition Gazebo using ROS 2 & MoveIt Task Constructor

## Overview

This project implements a perception-to-motion pipeline for the academy robot in Ignition Gazebo using ROS 2 Humble and MoveIt Task Constructor (MTC).

The intended assignment architecture was:

1. detect the screw from the simulated RGBD sensor,
2. estimate its 6D pose,
3. expose that pose through a ROS 2 topic and service,
4. call an MTC action server,
5. plan and execute a pick-and-place style task in Gazebo.

The final repository contains a working end-to-end path, but the action server was simplified during debugging to obtain a stable and repeatable plan/execute cycle. Therefore, the final implementation demonstrates the perception pipeline, custom ROS interfaces, action feedback, and Gazebo execution successfully, while the MTC manipulation logic remains a practical simplified version rather than a full grasp-lift-transport-place pipeline.

---

## Project Environment

### Host workspace root
~/academy_robotics-main/ros2_academy_ws

### Inside Docker
/home/user/ros2_ws

### Important workspace packages
- academy_robot_interfaces
- academy_robot_moveit_config
- academy_robot_simulation
- academy_robot_description
- academy_robot_sensors
- Universal_Robots_ROS2_Description

### Important simulation package
- academy_robot_gazebo_ignition
- launch/spawn_world.launch.py
- launch/spawn_robot.launch.py
- meshes/socket_cap_screw.stl
- worlds/socket_cap_screw.sdf

---

## Repository Structure

### academy_robot_pick_place
- src/object_detector_node.cpp
- src/pick_place_server.cpp
- src/pick_place_client_node.cpp
- CMakeLists.txt
- package.xml

### academy_robot_interfaces
- srv/DetectObject.srv
- action/PickPlace.action

### Important modified external launch file
- /home/user/ros2_ws/src/academy_robot_simulation/academy_robot_gazebo_ignition/launch/spawn_robot.launch.py

---

## Implemented ROS 2 Interfaces

### Subscribed simulation topics
- /front_rgbd_camera/color/image_raw
- /front_rgbd_camera/depth/image_raw
- /front_rgbd_camera/depth/camera_info
- /front_rgbd_camera/depth/color/points
- /joint_states

### Implemented interfaces
- /detected_object_pose → geometry_msgs/msg/PoseStamped
- /detect_object → academy_robot_interfaces/srv/DetectObject
- /pick_place_task → academy_robot_interfaces/action/PickPlace

### DetectObject.srv
---
bool success
geometry_msgs/PoseStamped detected_pose

### PickPlace.action
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

## Architecture Overview

The final architecture is:

Ignition Gazebo RGBD point cloud
→ object_detector_node
→ /detected_object_pose topic + /detect_object service
→ pick_place_client_node
→ /pick_place_task action
→ pick_place_server
→ MoveIt / move_group / MTC
→ Gazebo execution through trajectory controllers

### Practical interpretation
The perception side is object-driven: the detector computes and caches the screw pose from the point cloud and exposes it through a service.

The manipulation side is action-based: the client requests the pose, builds a PickPlace goal, and sends it to the MTC server.

The final stable action server uses a simplified named-state pipeline that was produced after many debugging iterations:
- current state
- move arm to pregrasp
- open hand
- move arm to place_pregrasp
- open hand

This means the final implementation demonstrates:
- object detection,
- ROS 2 interfaces,
- action feedback,
- planning/execution timing,
- and Gazebo arm motion,

but it is not a full final grasp/lift/transport/place manipulation stack.

---

## Task-by-Task Implementation Summary

# Phase 1 — Object Detection Node

## T1 — Subscribe to camera topics

### What was implemented
object_detector_node was created and the main perception input became:

- /front_rgbd_camera/depth/color/points

This point cloud topic was used instead of manually fusing:
- RGB image,
- depth image,
- and camera intrinsics.

### Why this approach was chosen
The assignment required 3D object isolation and 6D pose estimation. Using PointCloud2 directly was the most reliable and shortest path to T2 and T3 because it already carries spatial information.

### Important note
During development, it initially looked as if the point cloud topic was not streaming correctly because:
- ros2 topic hz did not always show useful output,
- the node logs only showed a first message in some runs,
- later versions of the detector logged more sparsely.

In practice, the topic was available and the detector did receive data correctly.

### Tradeoff
- Pros: simpler 3D pipeline, avoids manual projection
- Cons: less emphasis on image-only processing

---

## T2 — Detect and isolate the object point cloud

T2 was one of the most important and most heavily debugged parts of the assignment.

### First approach: color-based filtering

#### What was tried
The first implementation attempted to segment the screw using point cloud RGB values.

Examples of tested parameters:
- target_r=240
- target_g=240
- target_b=240

We also tested looser color thresholds and a white-object heuristic using:
- use_white_heuristic=true
- white_min_channel
- white_channel_spread

#### Why this was tried
The screw looked visually white/light in the simulation, so color filtering seemed like a natural first strategy.

#### What failed
Diagnostic logs showed this did not isolate the object correctly:
- after depth filtering, average RGB values in relevant points were closer to about 100–113,
- target_match=0,
- white_like=0.

So even though the screw looked bright in the rendered scene, the point cloud RGB values did not support simple white thresholding.

#### Why it failed
The bridged point cloud RGB data did not match intuitive rendered appearance reliably enough for threshold-based segmentation.

#### Conclusion
Color-based segmentation was abandoned because it was brittle and repeatedly produced zero isolated points.

### Final approach: geometry-based seed clustering

#### What changed
The detector switched to a geometry-based method because the screw is:
- static,
- the only target object,
- on the floor,
- visible in a constrained region of the camera view.

#### Final method
1. restrict search to an image ROI in the organized point cloud,
2. find the nearest valid depth point in that ROI as a seed,
3. grow a local object cluster around the seed using:
   - a pixel neighborhood,
   - a Euclidean cluster radius,
   - and a depth margin relative to the seed,
4. publish the isolated cluster.

#### Important parameters that worked
min_depth_m=0.0
max_depth_m=10.0
roi_u_min_ratio=0.20
roi_u_max_ratio=0.80
roi_v_min_ratio=0.35
roi_v_max_ratio=1.00
seed_depth_margin_m=0.06
cluster_radius_m=0.10
region_radius_px=140
min_points_for_object=20

#### Important intermediate failure
Earlier versions used depth limits that were too strict, so no valid seed was found.

A key debugging discovery was that the useful point cloud region had camera-frame distances around 6 m, not the much smaller values originally assumed. That is why the final detector uses:
- max_depth_m=10.0

#### Example successful T2 output
isolated=153 points
seed_px=(1024,359)
seed_xyz=(6.0778, -3.3577, 0.0044)
centroid=(6.0778, -3.3577, 0.0295)

#### Why this approach was chosen
It was much more robust than color filtering for this exact scene and assignment.

#### Tradeoffs
Pros
- robust in this simulation,
- does not depend on unreliable point cloud RGB,
- easy to explain and tune.

Cons
- assumes one nearest target object in a known region,
- less general than a full object segmentation pipeline,
- tailored to this environment rather than universal.

---

## T3 — Extract 6D object pose

T3 was the second major perception task and required several practical adjustments.

### Original intended idea
The assignment direction suggested 6D pose estimation through registration against a known object model. That idea was kept, but the implementation changed based on available dependencies.

### What was attempted first
We checked whether Open3D was available in the environment, because that would have been a convenient tool for registration.

### What failed
Open3D was not available in the environment:
- pkg-config Open3D failed,
- package lookups did not show it installed.

### Final approach: PCL ICP using an STL-derived reference point cloud

#### What was implemented
1. load the screw STL:
   /home/user/ros2_ws/src/academy_robot_simulation/academy_robot_gazebo_ignition/meshes/socket_cap_screw.stl
2. convert the STL mesh into a point cloud,
3. center the reference cloud at its centroid,
4. downsample reference and observed clouds,
5. align them using PCL ICP,
6. extract translation and rotation from the final transform,
7. convert rotation matrix to quaternion,
8. build a PoseStamped in the camera frame,
9. transform it into robot_odom using tf2.

### Main problems encountered in T3

#### 1) Wrong target frame assumption
At first, the detector attempted to transform to world.

This failed because world did not exist as a usable TF target in the actual running graph.

#### Fix
After inspecting TF, robot_odom was selected as the final target frame.

#### 2) Reference scale mismatch
The STL-derived reference cloud was initially at the wrong scale relative to the observed point cloud.

This caused:
- too many reference points,
- poor overlap,
- not enough useful ICP correspondences.

#### Fix
The key correction was:
reference_scale=0.001

This indicates the reference mesh needed millimeter-to-meter scaling.

#### 3) ICP tuning and point density
Before tuning, the downsampled clouds were not well matched for ICP.

#### Final useful parameters
reference_scale=0.001
voxel_leaf_size_m=0.0015
icp_max_corr_m=0.03
icp_max_iterations=80
icp_fitness_threshold=0.02

These values produced stable alignment for the screw in this simulation.

#### 4) TF readiness and timing
Some early attempts failed because TF to robot_odom was not yet available when the node first processed data.

#### Fix
Later versions used TF lookup/canTransform logic and cached successful transformed poses. Once TF became available, T3 succeeded on later frames.

### Example successful T3 output
camera_frame=robot_front_rgbd_camera_depth_frame
target_frame=robot_odom
fitness=0.000006
camera_xyz=(6.0777, -3.3577, 0.0277)
global_xyz=(6.4462, -3.3377, 0.4053)

### Why this approach was chosen
- it follows the assignment’s registration-based spirit,
- it uses the provided screw STL,
- it was feasible with available dependencies because PCL was present.

### Tradeoffs
Pros
- true registration-based pose estimation,
- produces full pose, not only centroid,
- uses provided object geometry.

Cons
- tuned to this screw and scene,
- sensitive to scale, overlap, and thresholds,
- less sophisticated than feature-based or global-registration pipelines.

---

## T4 — Publish and serve the pose

T4 was implemented as both a topic publisher and a service.

### Implemented output interfaces
- topic: /detected_object_pose
- service: /detect_object

### Final behavior
The detector continuously publishes the latest valid PoseStamped on /detected_object_pose.

At the same time, it caches the latest valid pose internally and returns it on demand through /detect_object.

### Service policy
- if a valid transformed pose exists:
  - return success=true
  - return cached detected_pose
- otherwise:
  - return success=false

### Why both topic and service were used
Topic
Useful for:
- continuous debugging,
- RViz inspection,
- monitoring detector state over time.

Service
Useful for:
- request/response logic from the client,
- avoiding indefinite subscriptions in the action client,
- cleaner T11 integration.

### Important design choice
The service does not rerun perception inside the callback. It returns the latest cached valid pose.

### Why this was chosen
It keeps the service:
- lightweight,
- responsive,
- decoupled from continuous perception.

### Tradeoff
Pros
- simple and fast service behavior,
- client logic is clean,
- continuous perception stays independent.

Cons
- returned pose is the latest known pose, not necessarily recomputed exactly at request time.

---

# Phase 2 — Custom ROS 2 Interfaces

## T5 — DetectObject.srv

Implemented in:
- academy_robot_interfaces/srv/DetectObject.srv

---
bool success
geometry_msgs/PoseStamped detected_pose

### Notes
- interfaces were placed in a separate ROS 2 interfaces package, which is the correct pattern,
- this service was later used successfully by the client in T11.

## T6 — PickPlace.action

Implemented in:
- academy_robot_interfaces/action/PickPlace.action

geometry_msgs/PoseStamped target_pose
geometry_msgs/PoseStamped place_pose
string object_id
---
bool success
int64 planning_time_ms
int64 execution_time_ms
---
string current_stage

### Notes
This action definition supported:
- target pose passing,
- place pose passing,
- action feedback,
- timing statistics.

---

# Phase 3 — MTC Pick & Place Action Server

## T7 — Set up the MTC task

### What was done
A dedicated action server node was created:
- src/pick_place_server.cpp

### Early issues
Initially, MoveIt Task Constructor packages were missing from the environment.

Packages installed during debugging included:
- moveit_task_constructor_core
- moveit_task_constructor_msgs
- later moveit_task_constructor_capabilities

### Build system issue
There was also an environment-specific CMake issue with assumed MTC target names.

### Final practical fix
The package was built using:
- ament_target_dependencies(... moveit_task_constructor_core ...)

without relying on broken explicit target_link_libraries assumptions.

## T8 — Build stage pipeline

### What happened
We originally tried building a more complex manipulation pipeline.

During debugging, we confirmed from the SRDF that the important groups/states were:
- arm group: ur_arm
- hand group: hand
- hand states: open, close

### Problems encountered
More ambitious stage pipelines ran into:
- invalid group names,
- stage interface mismatch errors,
- task.init() failures,
- stage connectivity problems,
- unstable execution behavior.

### Final tradeoff
The pipeline was simplified multiple times until it became stable enough to:
- initialize,
- plan,
- execute,
- and return success.

### Final working pipeline
The final stable pipeline is:

1. current state
2. move arm to pregrasp
3. open hand
4. move arm to place_pregrasp
5. open hand

### Important honesty note
This is a working simplified MTC pipeline, not the full ideal pick-lift-transport-place pipeline originally imagined by the assignment.

## T9 — Stream stage feedback

### What was implemented
The action server publishes current_stage feedback throughout execution.

### Example feedback stages
- goal_received
- task_initializing
- adding_current_state
- adding_pregrasp
- adding_open_hand_pregrasp
- adding_place_pregrasp
- adding_open_hand_release
- pipeline_built
- pipeline_initializing
- pipeline_initialized
- planning_started
- planning_succeeded
- execution_started
- execution_succeeded
- execution_failed

### Result
The client later received and printed these feedback messages correctly.

## T10 — Plan and execute

T10 required many debugging iterations.

### Major initial problems
- /execute_task_solution did not exist,
- move_group was not exposing ExecuteTaskSolutionCapability,
- some gripper stage combinations hung,
- some runs rejected goals,
- some runs failed with start-state issues,
- some more complex stage graphs were valid in theory but unstable in practice.

### Important launch fix
The key external change was modifying spawn_robot.launch.py so move_group loads:
- move_group/ExecuteTaskSolutionCapability
- trajectory_execution.execution_duration_monitoring = True
- trajectory_execution.allowed_execution_duration_scaling = 1.2
- trajectory_execution.allowed_goal_duration_margin = 0.5
- trajectory_execution.allowed_start_tolerance = 0.01

### Why this mattered
Without this, MTC could not hand off task execution correctly to move_group.

### Final working T10 interpretation
The final implementation proves that:
- the action framework works,
- MTC task initialization works,
- planning timing is measured,
- execution timing is measured,
- Gazebo execution succeeds.

However, it is important to document that the final stable version is the simplified action server pipeline reached after pragmatic debugging, not a fully featured manipulation solution.

---

# Phase 4 — Pick & Place Client Node

## T11 — Call the detection service

### What worked
The client successfully called:
- /detect_object

and received:
- success=true
- a pose in robot_odom

### Example
xyz=(6.4462, -3.3377, 0.4053)

## T12 — Send the action goal

### What worked
The client built a PickPlace goal from:
- target_pose returned by /detect_object
- a configured place_pose in robot_odom
- object_id=socket_cap_screw

The goal was accepted by:
- /pick_place_task

## T13 — Log progress and handle result

### What worked
The client:
- received feedback through the action feedback callback,
- waited for the final result,
- logged final timing and status.

### Successful end-to-end behavior
The current codebase has a working end-to-end run for the simplified pipeline:
- detection succeeded,
- goal was accepted,
- feedback stages streamed,
- execution succeeded,
- final status returned SUCCEEDED.

---

## Failed Approaches and Lessons Learned

### 1) Color-based object detection failed
The screw looked visually white, but the point cloud RGB values did not support simple thresholding. Result: segmentation by color was abandoned.

### 2) Too-restrictive depth assumptions failed
The relevant points were much farther than originally assumed. The detector only became reliable when max_depth_m was increased to 10.0.

### 3) Open3D-based registration was not possible
Open3D was not present in the environment, so PCL ICP became the practical alternative.

### 4) Wrong TF target frame failed
world was not usable. robot_odom was the correct frame in practice.

### 5) Large/complex MTC pipelines were unstable in practice
Several richer manipulation pipelines were attempted, but repeated interface mismatches, init failures, and execution instability forced a simplification.

### 6) Repeated gripper stages were problematic
Some intermediate action server versions with extra hand stages could hang during execution. The final stable version removed the problematic close-hand stage and kept the simpler sequence.

---

## Design Tradeoffs

### Perception tradeoffs
- chose direct PointCloud2 over RGB+depth reconstruction,
- chose geometry-based seed clustering over color segmentation,
- chose PCL ICP over unavailable Open3D,
- chose cached latest pose for the service rather than rerunning detection inside the callback.

### Manipulation tradeoffs
- chose a simpler stable MTC stage sequence over a more ambitious but unstable full manipulation graph,
- prioritized a reliable end-to-end demo path over full ideal pick-and-place semantics.

### Why these tradeoffs were reasonable
This assignment required a working integration across:
- simulation,
- perception,
- interfaces,
- actions,
- MoveIt,
- and Gazebo execution.

Under time and environment constraints, reliability and debuggability were prioritized over full generality.

---

## Current Final Status

### What is clearly working
- T2 object isolation works robustly in the target scene
- T3 pose extraction works with STL-based ICP
- T4 topic publishing and service response work
- T5/T6 interfaces are implemented
- T11/T12/T13 work end-to-end
- T10 works for the final simplified pipeline
- Gazebo execution is demonstrated successfully

### Honest summary of the final implementation
The project now has a working end-to-end path, but the final MTC action server was simplified during debugging to keep the system reliable.

So the repository should be interpreted as:
1. a correct and well-documented perception pipeline,
2. a correct ROS 2 interface and action architecture,
3. a practical simplified manipulation demo that executes successfully in Gazebo.

---

## Known Limitations

1. The final MTC pipeline is simplified
   - it is not yet a full grasp / lift / transport / place pipeline.

2. Detection is tailored to this simulation
   - ROI and cluster assumptions are specific to this environment.

3. ICP is object-specific
   - the registration is tuned for the provided screw STL and current scene geometry.

4. Service returns latest cached pose
   - the service does not force a fresh recomputation at request time.

5. Execution duration can vary
   - some successful runs are fast, while others take much longer depending on the debugging version and execution path.

---

## Future Improvements

If this project were extended, the most useful next steps would be:

1. replace the simplified named-state MTC pipeline with a full manipulation sequence:
   - pre-grasp
   - grasp
   - lift
   - transport
   - place

2. use the action goal poses more directly inside manipulation logic rather than relying only on named joint states,

3. improve generality of T2 by using more robust segmentation rather than scene-specific ROI/nearest-point assumptions,

4. improve T3 with richer registration or better initialization to reduce parameter sensitivity,

5. add stronger controller-state validation and more polished launch-time synchronization.

---

## How to Run the Final Working Version

## 1) Launch the world
ros2 launch academy_robot_gazebo_ignition spawn_world.launch.py spawn_screw:=true

Important:
spawn_screw:=true is required. Without it, the screw will not appear.

## 2) Launch the robot
ros2 launch academy_robot_gazebo_ignition spawn_robot.launch.py robot:=academy_robot robot_model:=academy_robot_plus has_arm:=true

## 3) Run the detector
ros2 run academy_robot_pick_place object_detector_node --ros-args   -p min_depth_m:=0.0   -p max_depth_m:=10.0   -p roi_u_min_ratio:=0.20   -p roi_u_max_ratio:=0.80   -p roi_v_min_ratio:=0.35   -p roi_v_max_ratio:=1.00   -p seed_depth_margin_m:=0.06   -p cluster_radius_m:=0.10   -p region_radius_px:=140   -p min_points_for_object:=20   -p reference_stl_path:=/home/user/ros2_ws/src/academy_robot_simulation/academy_robot_gazebo_ignition/meshes/socket_cap_screw.stl   -p target_frame:=robot_odom   -p reference_scale:=0.001   -p voxel_leaf_size_m:=0.0015   -p icp_max_corr_m:=0.03   -p icp_max_iterations:=80   -p icp_fitness_threshold:=0.02   -p tf_lookup_timeout_s:=1.0   -p pose_publish_period_ms:=500

Expected pose example:
frame=robot_odom
xyz=(6.4462, -3.3377, 0.4053)

## 4) Run the action server
ros2 run academy_robot_pick_place pick_place_server

## 5) Run the client
ros2 run academy_robot_pick_place pick_place_client_node --ros-args -p result_timeout_sec:=420.0

### Why the client timeout is large
Some successful runs took several minutes to finish, so a larger timeout is safer for the working final version.

---

## How to Explain This Project in a Viva / Report

### Short explanation
This project implements a perception-driven pick-and-place architecture in ROS 2 Humble. The detector isolates the screw from the RGBD point cloud, estimates its 6D pose by ICP registration against the provided STL model, publishes that pose, and serves it through a ROS 2 service. The client requests the pose and sends a PickPlace action goal to an MTC action server, which plans and executes a simplified manipulation task in Gazebo while streaming feedback.

### Honest technical explanation
The perception stack is stronger and more complete than the final manipulation stack. T2, T3, and T4 were implemented carefully and validated. On the manipulation side, several richer MTC pipelines were attempted, but the final submitted implementation uses a simplified named-state pipeline because it was the most stable end-to-end solution under the available environment and time constraints.

### Strong points to emphasize
- the detector is geometry-based and does not rely on unreliable simulated RGB thresholds,
- the pose estimation is registration-based and uses the provided screw STL,
- the ROS 2 interfaces are correctly separated into topic, service, and action,
- feedback, planning timing, and execution timing are all integrated.

---

## Final Takeaway

The final repository demonstrates a working perception-to-execution path in ROS 2 Humble and Ignition Gazebo.

Its biggest strengths are:
- robust object isolation for this simulation,
- registration-based pose extraction,
- clean ROS 2 interface design,
- and successful end-to-end action execution.

Its biggest limitation is that the final MTC server is a simplified stable pipeline rather than a full final manipulation solution.
