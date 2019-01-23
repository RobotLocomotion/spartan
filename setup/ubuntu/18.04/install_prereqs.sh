#!/bin/bash

set -euxo pipefail

apt-get update

# Noninteractive frontend required to
# get tzdata install to not ask for some
# location info.
DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends \
  cmake \
  cmake-curses-gui \
  freeglut3-dev \
  libboost-all-dev \
  libhdf5-dev \
  libjsoncpp-dev \
  libnetcdf-dev \
  libnetcdf-c++4-dev \
  libnetcdf-cxx-legacy-dev \
  libogg-dev \
  libopencv-dev \
  libopenni2-dev \
  libqhull-dev \
  libqt5opengl5-dev \
  libqt5x11extras5-dev \
  libqwt-qt5-dev \
  libtheora-dev \
  libusb-1.0-0-dev \
  libxt-dev \
  libtheora-dev \
  libxml2-dev \
  make \
  python-matplotlib \
  qt5-default \
  qtbase5-private-dev \
  qtmultimedia5-dev \
  qtscript5-dev \
  qttools5-dev \
  libcgal-dev \
  libxmu-dev \
  libxi-dev


# Install ROS Melodic
apt install -y --no-install-recommends lsb-release
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

apt-get update

# Noninteractive frontend required to
# get tzdata install to not ask for some
# location info. Not sure if it's installed
# in this apt install or the above.
DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
  ros-melodic-desktop \
  ros-melodic-moveit \
  ros-melodic-openni2-launch \
  ros-melodic-image-pipeline \
  ros-melodic-perception-pcl
