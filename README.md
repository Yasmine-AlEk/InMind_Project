# ROS2 Academy Robotics Workspace

This repository contains a modular ROS 2 workspace for simulation and development of mobile robots, including custom robot descriptions, simulation environments, and interface messages.

## Table of Contents

- [ROS2 Academy Robotics Workspace](#ros2-academy-robotics-workspace)
  - [Table of Contents](#table-of-contents)
  - [Workspace Structure](#workspace-structure)
  - [Requirements](#requirements)
    - [System Dependencies](#system-dependencies)
  - [Installation](#installation)
  - [Building the Workspace](#building-the-workspace)
  - [Usage \& Launch Commands](#usage--launch-commands)
    - [1. **Gazebo Ignition (Gazebo Sim) Simulation**](#1-gazebo-ignition-gazebo-sim-simulation)
      - [Spawn World](#spawn-world)
      - [Spawn Robot](#spawn-robot)
      - [Teleoperation (Keyboard)](#teleoperation-keyboard)
      - [Joint Trajectory Controller (RQT)](#joint-trajectory-controller-rqt)
    - [2. **Universal Robots Description**](#2-universal-robots-description)
  - [Packages Overview](#packages-overview)
---

## Workspace Structure

```
ros2_academy_ws/
├── src/
│   ├── academy_robot_common/
│   ├── academy_robot_description/
│   ├── academy_robot_interfaces/
│   ├── academy_robot_sensors/
│   ├── academy_robot_simulation/
│   └── Universal_Robots_ROS2_Description/
├── build/
├── install/
├── log/
├── README.md
└── ...
```

## Requirements

- **OS:** Ubuntu 22.04
- **ROS 2:** Humble Hawksbill

### System Dependencies (2 Ways)

#### 1. Dockerized Application (Recommended)

The recommended way to build and run the application executables is using the docker container. 
Follow [docker readme](docker/README.md) for full docker setup.


#### 2. Install Locally

**ROS2 and other dependencies on your local Ubuntu machine:**
Install ROS 2 Humble (see [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)).

**Required ROS 2 packages:**
```sh
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ament-cmake \
  ros-humble-ament-lint-auto \
  ros-humble-ament-cmake-python \
  ros-humble-launch-testing \
  ros-humble-ament-cmake-pytest \
  ros-humble-rqt-joint-trajectory-controller 
```
**Install the robot controller:**
```sh
cd ros2_academy_ws/src/academy_robot_simulation/debs
sudo dpkg -i ros-humble-robotnik-controllers_0.0.0-20250407.104325-b0a866a_amd64.deb
```

**Other recommended tools:**
```sh
sudo apt install -y git wget curl
```

**Initialize rosdep:**
```sh
sudo rosdep init
rosdep update
```

**Installation**

Clone this repository and initialize submodules if any:
```sh
git clone <repo-url> ros2_academy_ws
cd ros2_academy_ws
```

Install dependencies using rosdep:
```sh
rosdep install --from-paths src --ignore-src -r -y
```

## Building the Workspace

```sh
cd ros2_academy_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Source the workspace after building:
```sh
source install/setup.bash
```

## Usage & Launch Commands

### 1. **Gazebo Ignition (Gazebo Sim) Simulation**

#### Spawn World
```sh
ros2 launch academy_robot_gazebo_ignition spawn_world.launch.py
```

#### Spawn World with Socket Cap Screw Object
```sh
ros2 launch academy_robot_gazebo_ignition spawn_world.launch.py spawn_screw:=true
```

#### Spawn Robot
```sh
ros2 launch academy_robot_gazebo_ignition spawn_robot.launch.py robot:=academy_robot robot_model:=academy_robot_plus has_arm:=true
```

#### Teleoperation (Keyboard)
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robotnik_base_controller/cmd_vel -p stamped:=true
```

#### Joint Trajectory Controller (RQT)
```sh
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller --ros-args -r __ns:=/robot
```

### 2. **Universal Robots Description**

View a Universal Robot in RViz:
```sh
ros2 launch ur_description view_ur.launch.py ur_type:=ur3
```
Replace `ur3` with any supported type: `ur3`, `ur3e`, `ur5`, `ur5e`, `ur7e`, `ur10`, `ur10e`, `ur12e`, `ur16e`, `ur15`, `ur20`, `ur30`.

## Packages Overview

- **academy_robot_common**: Common launch and utility scripts.
- **academy_robot_description**: Robot URDF/Xacro, meshes, and description launch files.
- **academy_robot_interfaces**: Custom ROS 2 messages and services.
- **academy_robot_sensors**: Sensor models and configuration.
- **academy_robot_simulation**
  - **academy_robot_gazebo_classic**: Gazebo Classic simulation launch and worlds.
  - **academy_robot_gazebo_ignition**: Gazebo Ignition (Sim) simulation launch and worlds.
- **Universal_Robots_ROS2_Description**: Universal Robots models and launch files.
