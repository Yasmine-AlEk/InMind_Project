# Academy ROS2 Docker Workspace

This repository provides a Docker-based environment for ROS2 Humble and Gazebo Ignition Fortress.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) installed on your system.
- [ros2_academy_ws](https://github.com/Jad-ELHAJJ/academy_robotics) locally cloned.

## Building the Docker Image

To build the Docker image, run the following command from the root of this repository:

```sh
./docker/build_image.sh
```

This script will build the Docker image using the provided Dockerfile and install all necessary dependencies.

## Starting the Docker Container

After building the image, start the container with:

```sh
./docker/start_container.sh
```

This script will launch the container with the required environment variables and volume mounts.  
If `ROS_DOMAIN_ID` is not set in your environment, the script will default it to `1`.

## Stopping the Container

To stop and remove the running container:

```sh
docker stop academy_robotics
docker rm academy_robotics
```

## Start a shell for the container

```sh
docker exec -it academy_robotics /bin/bash
```
