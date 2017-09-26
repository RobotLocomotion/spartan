#!/bin/bash

set -euxo pipefail

add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
apt update
apt install --no-install-recommends \
  libboost-all-dev \
  libjsoncpp-dev \
  libogg-dev \
  libopencv-dev \
  libopenni2-dev \
  libqhull-dev \
  libqt4-opengl-dev \
  libqwt-dev \
  libtheora-dev \
  libusb-1.0-0-dev \
  libxt-dev \
  python-matplotlib \
  qt4-default

if [ ! -f /etc/profile.d/cmake.sh ]; then
  pushd /tmp
  curl -O https://cmake.org/files/v3.5/cmake-3.5.2-Linux-x86_64.tar.gz
  tar -xzf cmake-3.5.2-Linux-x86_64.tar.gz
  mv cmake-3.5.2-Linux-x86_64 /opt/cmake
  rm cmake-3.5.2-Linux-x86_64.tar.gz
  popd
  echo 'export PATH="/opt/cmake/bin:$PATH"' > /etc/profile.d/cmake.sh
fi
