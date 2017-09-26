#!/bin/bash

set -euxo pipefail

apt install --no-install-recommends \
  libboost-all-dev \
  libjsoncpp-dev \
  libtheora-dev \
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
  python-matplotlib \
  qt5-default \
  qtbase5-private-dev \
  qtmultimedia5-dev \
  qtscript5-dev \
  qttools5-dev
