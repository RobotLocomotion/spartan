#!/usr/bin/env python

import argparse
import os

if __name__=="__main__":
	print "building docker container . . . "
	parser = argparse.ArgumentParser()
	parser.add_argument("-i", "--image", type=str,
		help="name for the newly created docker image", default="spartan")

	parser.add_argument("-d", "--dry_run", action='store_true', help="(optional) perform a dry_run, print the command that would have been executed but don't execute it.")

	args = parser.parse_args()
	print "building docker image named ", args.image
	cmd = "docker build -t %s -f setup/docker/spartan.dockerfile ." % args.image
	

	print "command = \n \n", cmd
	print ""

	# build the docker image
	if not args.dry_run:
		print "executing shell command"
		os.system(cmd)
	else:
		print "dry run, not executing command"