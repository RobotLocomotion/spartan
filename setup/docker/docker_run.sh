#!/bin/bash
#
# Usage:  ./docker_run.sh 
#

image_name=spartan-tmp

source_dir=$(pwd)

xhost +local:root;
nvidia-docker run -it -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $source_dir:/root/spartan $image_name
xhost -local:root;