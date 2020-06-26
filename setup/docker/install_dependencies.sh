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




# added catkin_pkg to deal with an error that arose during rospy_message_converter installation
# https://stackoverflow.com/questions/47992442/importerror-no-module-named-catkin-pkg-packages
# addirng catkin_pkg didn't seem to fix the issue, will try to go on without it for now . . .
pip install \
  numpy \
  scipy \
  pyassimp \
  pyglet \
  plyfile \
  matplotlib==1.5.3 \
  scikit-image \
  pytest-xdist \
  trimesh \
  # imgaug \


# imgaug erroring out due to wanting Python >= 3.5. Maybe should pin to old version
# of imgaug . . . do we even need imgaug? Probably not
# Collecting imageio (from imgaug)
#   Downloading https://files.pythonhosted.org/packages/ea/59/c6d568a309cf4c1b8e091bd8630c590a57f2e068a8e4114e1f36ee0d8e4f/imageio-2.8.0.tar.gz (3.3MB)
# imageio requires Python '>=3.5' but the running Python is 2.7.12
# You are using pip version 9.0.3, however version 20.1.1 is available.
# You should consider upgrading via the 'pip install --upgrade pip' command.

# You should consider upgrading via the 'pip install --upgrade pip' command.
# maybe this should be in the ros section?
# pip install rospy_message_converter

