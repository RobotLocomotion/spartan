#!/bin/bash

set -euxo pipefail
root_dir=$(pwd)

mkdir handical_dependencies
workdir=root_dir/handical_dependencies

apt-get update
apt install --no-install-recommends \
  libopencv-dev \
  python-opencv

install_gtsam()
{   cd $workdir
    git clone https://thduynguyen@bitbucket.org/thduynguyen/gtsam-duy.git
    cd gtsam-duy
    gtsam_source_dir = $(pwd)
    git fetch && git checkout feature/wrap_cal3ds2
    pip install -r cython/requirements.txt # install cython requirements

    mkdir build
    cd build
    mkdir install
    gtsam_install_dir=$gtsam_source_dir/build/install
    cmake -DGTSAM_INSTALL_CYTHON_TOOLBOX=ON -DCMAKE_INSTALL_PREFIX=$gtsam_install_dir ..
    make -j4
    make install -j4

    echo export GTSAM_INSTALL_DIR=$gtsam_install_dir >> ~/.bashrc
}

install_gtsam