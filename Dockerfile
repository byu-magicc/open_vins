# Base image
FROM docker.io/osrf/ros:humble-desktop-full

# Update system
RUN apt-get update && apt-get upgrade -y && rosdep update

# Ceres solver install and setup
RUN apt-get install -y cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev libceres-dev

# ROS dependencies
WORKDIR /open_vins_ws
COPY . src/open_vins
RUN rosdep install --from-path . -y --ignore-src

# Remove apt and rosdep cache
RUN rm -rf /var/lib/apt/lists/* /root/.ros/rosdep

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Add workspace source file to entrypoint
RUN sed -i '/exec "\$@"/i source "\/open_vins_ws\/install\/setup.bash" --' /ros_entrypoint.sh
