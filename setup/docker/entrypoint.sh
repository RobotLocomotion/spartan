#!/bin/bash
set -e

function use_spartan()
{
	. ~/spartan/build/setup_environment.sh
}

export -f use_spartan

exec "$@"

cd ~/spartan
echo $2 | sudo -S bash -c 'echo "'$1' ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/docker-user' && printf "\n"
