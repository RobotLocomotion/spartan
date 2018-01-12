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

function use_cpf()
{
    . ~/spartan/src/ContactParticleFilter/setup/setup_environment_cpf.sh
}
export -f use_spartan
export -f use_ros
export -f use_spartan_ros
export -f use_cpf

exec "$@"

cd ~/spartan
