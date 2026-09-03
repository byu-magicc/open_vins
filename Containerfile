# Base image
FROM docker.io/osrf/ros:jazzy-desktop-full
ENV DEBIAN_FRONTEND=noninteractive \
  PIP_NO_CACHE_DIR=1 \
  LANG=C.UTF-8 \
  LC_ALL=C.UTF-8

# Update packages and install system dependencies
RUN apt-get update && apt-get upgrade -y && \
  apt-get install -y --no-install-recommends \
  # gtsam dependencies
  build-essential cmake git apt-utils ca-certificates \
  libeigen3-dev libboost-all-dev \
  # open_vins dependencies
  libgflags-dev libatlas-base-dev libsuitesparse-dev libceres-dev \
  python3-matplotlib python3-numpy python3-scipy python3-yaml && \
  rm -rf /var/lib/apt/lists/*

# Clone GTSAM
WORKDIR /
RUN git clone --branch 4.2.2 --depth 1 https://github.com/borglab/gtsam.git

# Build & install GTSAM
WORKDIR /gtsam/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && \
  make -j"$(nproc)" && make install && ldconfig

# Copy OpenVINS
WORKDIR /open_vins_ws
COPY . src/open_vins

# Build workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Add workspace source file to entrypoint
RUN sed -i '/exec "\$@"/i source "\/open_vins_ws\/install\/setup.bash" --' /ros_entrypoint.sh
