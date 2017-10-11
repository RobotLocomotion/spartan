#!/bin/bash
set -e

function use_spartan()
{
	. ~/spartan/build/setup_environment.sh
}

function use_ros()
{
	. /opt/ros/kinetic/setup.sh
}

export -f use_spartan
export -f use_ros

exec "$@"

cd ~/spartan
