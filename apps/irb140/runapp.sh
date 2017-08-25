#!/bin/bash

cd $SPARTAN_SOURCE_DIR/apps/irb140
drake-visualizer --script irb140ManipApp.py --bot-config irb140.cfg --director-config $SPARTAN_SOURCE_DIR/models/IRB140/director/director_config.json $*
