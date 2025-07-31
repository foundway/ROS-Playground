# https://hub.docker.com/_/ros

# Reference: https://github.com/osrf/docker_images/blob/master/ros/jazzy/ubuntu/noble/desktop-full/Dockerfile
# Base: https://hub.docker.com/layers/library/ros/jazzy-ros-base/images/sha256-50a4769126c5ace8e2cf5ba2acacd338fca425149e1ee0bed0cdcb53ffbf429c
FROM ros:jazzy-ros-base 

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop-full=0.11.0-1* \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

ARG username=$GIT_USERNAME
ARG password=$GIT_PASSWORD

ENV ROS_DISTRO=jazzy

WORKDIR /main_ws

# RUN pip3 install --no-cache-dir \
#     readchaV

# Expose ports for ROS2 communication
EXPOSE 11311
