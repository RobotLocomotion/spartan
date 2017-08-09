#!/bin/bash
set -e -x -u
add-apt-repository -y ppa:v-launchpad-jochen-sprickerhof-de/pcl 
apt-get update 
apt install \
  build-essential \
  cmake \
  libeigen3-dev \
  libboost-all-dev \
  libopenni2-dev \
  libopencv-dev \
  libqhull-dev \
  libqt4-dev \
  libqwt-dev \
  libusb-1.0-0-dev \
  libvtk5-dev \
  libvtk5-qt4-dev \
  python-dev \
  python-matplotlib \
  python-numpy \
  python-pip \
  python-scipy \
  python-vtk \
  python-yaml \
