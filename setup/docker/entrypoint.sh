#!/bin/bash
set -e

function use_spartan()
{
	. ~/spartan/build/setup_environment.sh
}

export -f use_spartan

exec "$@"

cd ~/spartan
