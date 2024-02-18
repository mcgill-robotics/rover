# Use the base image from ROS
FROM ros:noetic-ros-base-focal

# Instal librealsense
#
# The following procedure is based on the installation guide from librealsense (from https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

# install required tools for librealsense
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    apt-transport-https

# fetch the librealsense server public key
RUN sudo mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
    | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

# add the librealsense server to apt-get sources
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
    https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" \
    | sudo tee /etc/apt/sources.list.d/librealsense.list

# install librealsense
RUN apt-get update && apt-get install -y --no-install-recommends \
    # librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop \
    ros-noetic-rosserial \
    ros-noetic-cv-bridge \
    ros-noetic-rosbridge-suite \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-image-geometry \
    && rm -rf /var/lib/apt/lists/*

# Port 9090 is used for rosbridge
EXPOSE 9090
