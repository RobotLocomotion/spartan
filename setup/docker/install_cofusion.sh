#!/bin/bash
#
# This script is run by the dockerfile during the docker build.
#

set -exu

root_dir=$(pwd)/CoFusion
install_dir=$root_dir/install
mkdir -p $root_dir
mkdir -p $install_dir


#apt update

#apt install -y \
#  libglew-dev \
#  libsuitesparse-dev \
#  libeigen3-dev

build_cofusion()
{
  cd $root_dir
  git clone https://github.com/gizatt/co-fusion.git CoFusion
  cd CoFusion
  git checkout gizatt_build_1604

  mkdir deps
  cd deps

  # waiting on PR #190 to be merged
  #git clone --depth=1 https://github.com/stevenlovegrove/Pangolin.git
  git clone --depth=1 https://github.com/zhang-xin/Pangolin.git
  git clone --depth=1 https://github.com/occipital/OpenNI2.git
  git clone --depth=1 https://github.com/martinruenz/densecrf.git
  git clone --depth=1 https://github.com/carlren/gSLICr.git

  wget https://github.com/Itseez/opencv/archive/3.1.0.zip
  unzip 3.1.0.zip
  rm 3.1.0.zip
  cd opencv-3.1.0
  mkdir -p build
  cd build
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="`pwd`/../install" \
    \
    `# OpenCV: (building is not possible when DBUILD_opencv_video/_videoio is OFF?)` \
    -DBUILD_opencv_flann=ON  \
    -DWITH_CUDA=OFF  \
    -DBUILD_DOCS=OFF  \
    -DBUILD_TESTS=OFF  \
    -DBUILD_PERF_TESTS=OFF  \
    -DBUILD_opencv_java=OFF  \
    -DBUILD_opencv_python2=OFF  \
    -DBUILD_opencv_python3=OFF  \
    -DBUILD_opencv_features2d=ON  \
    -DBUILD_opencv_calib3d=ON  \
    -DBUILD_opencv_objdetect=ON  \
    -DBUILD_opencv_stitching=OFF  \
    -DBUILD_opencv_superres=OFF  \
    -DBUILD_opencv_shape=OFF  \
    -DWITH_1394=OFF  \
    -DWITH_GSTREAMER=OFF  \
    -DWITH_GPHOTO2=OFF  \
    -DWITH_MATLAB=OFF  \
    -DWITH_TIFF=OFF  \
    -DWITH_VTK=OFF  \
    \
    `# OpenCV-Contrib:` \
    -DBUILD_opencv_surface_matching=ON \
    -DBUILD_opencv_aruco=OFF \
    -DBUILD_opencv_bgsegm=OFF \
    -DBUILD_opencv_bioinspired=OFF \
    -DBUILD_opencv_ccalib=OFF \
    -DBUILD_opencv_contrib_world=OFF \
    -DBUILD_opencv_datasets=OFF \
    -DBUILD_opencv_dnn=OFF \
    -DBUILD_opencv_dpm=OFF \
    -DBUILD_opencv_face=OFF \
    -DBUILD_opencv_fuzzy=OFF \
    -DBUILD_opencv_line_descriptor=OFF \
    -DBUILD_opencv_matlab=OFF \
    -DBUILD_opencv_optflow=OFF \
    -DBUILD_opencv_plot=OFF \
    -DBUILD_opencv_reg=OFF \
    -DBUILD_opencv_rgbd=OFF \
    -DBUILD_opencv_saliency=OFF \
    -DBUILD_opencv_stereo=OFF \
    -DBUILD_opencv_structured_light=OFF \
    -DBUILD_opencv_text=OFF \
    -DBUILD_opencv_tracking=OFF \
    -DBUILD_opencv_xfeatures2d=OFF \
    -DBUILD_opencv_ximgproc=OFF \
    -DBUILD_opencv_xobjdetect=OFF \
    -DBUILD_opencv_xphoto=OFF \
    ..
  make -j8
  make install > /dev/null
  cd ../build
  OpenCV_DIR=$(pwd)
  cd ../..

  ./Scripts/install.sh

  export CMAKE_PREFIX_PATH=$install_dir
  mkdir build
  cd build
  cmake ../
  make -j$(nproc)

  ln -s $(pwd)/ElasticFusion $install_dir/bin

  # cleanup to make the docker image smaller
  find . -name \*.o | xargs rm

  # will this persist past image creation?
  export COFUSION_EXECUTABLE=$install_dir/CoFusion
}

build_cofusion