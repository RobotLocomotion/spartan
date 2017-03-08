#!/bin/bash

cd $(dirname $(realpath $0))
drake-visualizer --script iiwaManipApp.py --bot-config iiwaManip.cfg --director-config $SPARTAN_SOURCE_DIR/models/iiwa/director/director_config.json $*
