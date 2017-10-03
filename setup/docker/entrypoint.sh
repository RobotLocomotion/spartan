#!/bin/bash
set -e

function use_spartan()
{
	. /root/spartan/build/setup_environment.sh
}

export -f use_spartan

exec "$@"