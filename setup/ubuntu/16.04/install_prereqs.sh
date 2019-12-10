#!/bin/bash

set -euxo pipefail

apt-get update

apt install --no-install-recommends \
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


# Install ROS Kinetic
apt install -y --no-install-recommends lsb-release
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt-get update

apt install -y --no-install-recommends --allow-unauthenticated \
  ros-kinetic-desktop \
  ros-kinetic-moveit \
  ros-kinetic-openni2-launch \
  ros-kinetic-image-pipeline \
  ros-kinetic-perception-pcl \
  ros-kinetic-ros-numpy
