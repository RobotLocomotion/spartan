#!/bin/bash
set -e

export SPARTAN_SANDBOX_DIR=~/sandbox
export POSER_SANDBOX_DIR=$SPARTAN_SANDBOX_DIR/poser
export DATA_DIR=~/data
export SPARTAN_DATA_DIR=$DATA_DIR/spartan


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
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GTSAM_INSTALL_DIR/lib
    export PYTHONPATH=$PYTHONPATH:$SPARTAN_SOURCE_DIR/src/handical/build/cython/handical
}

function use_runner()
{
	export PYTHONPATH=$PYTHONPATH:~/spartan/build/catkin_projects/pyrunner_ros
}

function kip()
{
	use_ros && use_spartan && kuka_iiwa_procman
}

export -f use_spartan
export -f use_ros
export -f use_spartan_ros
export -f use_handical
export -f kip
export -f use_runner


exec "$@"

cd ~/spartan
