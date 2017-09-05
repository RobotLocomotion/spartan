#!/bin/bash

cd $SPARTAN_SOURCE_DIR/apps/openni_frame_grabber
drake-visualizer --script openni_frame_grabber_app.py --bot-config frame_grabber.cfg $*
