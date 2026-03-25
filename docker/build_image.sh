#!/bin/bash

# Used for building docker image on cross platform
# docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

DOCKER_BUILDKIT=1 docker build \
  --network host \
  --build-arg ros_distro=humble \
  -t academy_robotics:latest  \
  -f docker/dockerfile $@ .