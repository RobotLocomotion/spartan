#!/usr/bin/env python

import os, sys, signal

ROOT_LOGS_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/CorlDev/data/logs_test/'

if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  subdirs = next(os.walk(ROOT_LOGS_DIR))[1]

  for subdir in subdirs:
    complete_path = ROOT_LOGS_DIR + subdir

    print "\n\n\n***************** STARTING SUBDIR %s *************" % subdir

    command = "python run_pose_estimation_on_coral_data.py fgr %s" % (
        complete_path
      )
    print "\n", command, "\n\n"
    os.system(command)

    command = "python run_pose_estimation_on_coral_data.py goicp %s" % (
        complete_path
      )
    print "\n", command, "\n\n"
    os.system(command)

    command = "python run_pose_estimation_on_coral_data.py super4pcs %s" % (
        complete_path
      )
    print "\n", command, "\n\n"
    os.system(command)

    command = "python run_pose_estimation_on_coral_data.py mip %s" % (
        complete_path
      )
    print "\n", command, "\n\n"
    os.system(command)
