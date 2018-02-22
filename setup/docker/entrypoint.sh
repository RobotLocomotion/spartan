#!/bin/bash
set -e

function use_spartan()
{
	. ~/spartan/build/setup_environment.sh
}

function use_ros()
{
	. /opt/ros/kinetic/setup.bash
}

function use_spartan_ros()
{
	. ~/spartan/build/catkin_projects/devel/setup.bash
}

function use_handical()
{
	GTSAM_INSTALL_DIR=~/handical_dependencies/gtsam-duy/build/install
	export PATH=$PATH:$GTSAM_INSTALL_DIR/bin
	export PYTHONPATH=$PYTHONPATH:$GTSAM_INSTALL_DIR/cython
}

export -f use_spartan
export -f use_ros
export -f use_spartan_ros
export -f use_handical

exec "$@"

cd ~/spartan
