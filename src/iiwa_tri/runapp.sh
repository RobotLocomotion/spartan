#!/bin/bash

cd $SPARTAN_SOURCE_DIR/src/iiwa_tri
drake-visualizer --script iiwaManipApp.py --bot-config iiwaManip.cfg --director-config $SPARTAN_SOURCE_DIR/models/iiwa/director/director_config.json $*
