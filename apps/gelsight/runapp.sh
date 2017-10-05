#!/bin/bash

cd $SPARTAN_SOURCE_DIR/apps/gelsight
drake-visualizer --script gelsightVizApp.py --bot-config iiwaManip.cfg --director-config $SPARTAN_SOURCE_DIR/models/iiwa/director/director_config.json $*