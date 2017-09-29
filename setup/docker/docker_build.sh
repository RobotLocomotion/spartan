#!/bin/bash
#
# This script runs docker build to create the spartan docker iamge
#

set -exu

docker build -t spartan-tmp -f setup/docker/spartan.dockerfile .
