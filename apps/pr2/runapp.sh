#!/bin/bash

cd $(dirname $(realpath $0))
drake-visualizer --script app.py --director-config $SPARTAN_SOURCE_DIR/models/pr2/director/director_config.json $*
