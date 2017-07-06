#!/usr/bin/env python

''' For every file below this that ends in *.vtp, 
    try to convert it to .ply. '''

import os, sys
import yaml
import numpy as np
import matplotlib.pyplot as plt
import datetime
import signal
import re
import hashlib
import argparse
from math import atan2

from director import transformUtils
from director.thirdparty import transformations

if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  all_vtps = [os.path.join(dp, f) for dp, dn, filenames in os.walk("./") for f in filenames if os.path.splitext(f)[1] == '.vtp']

  for vtp_file in all_vtps:
    new_filename = os.path.splitext(vtp_file)[0] + ".ply"
    print "\nConverting %s to %s..." % (vtp_file, new_filename)

    command = "directorPython ../scripts/convertPolydata.py %s %s" % (
        vtp_file, new_filename
      )
    print command
    os.system(command)