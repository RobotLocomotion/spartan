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
  dialog