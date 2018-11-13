#!/bin/bash

set -euxo pipefail

apt-get update
apt install --no-install-recommends \
  terminator \
  tmux \
  nano \
  gedit \
  git \
  openssh-client \
  unzip \
  htop \
  libopenni-dev \
  apt-utils \
  usbutils \
  dialog \
  python-pip \
  python-pytest \
  libav-tools \
  openscad

# these following three are ElasticFusion dependencies
apt install --no-install-recommends \
  libglew-dev \
  libsuitesparse-dev \
  libeigen3-dev

pip install --upgrade pip==9.0.3
pip install -U setuptools

pip install \
  numpy \
  scipy \
  pyassimp \
  pyglet \
  plyfile \
  matplotlib==1.5.3 \
  scikit-image \
  pytest-xdist \
  vispy