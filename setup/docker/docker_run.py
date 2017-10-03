#!/usr/bin/env python

import argparse
import os

if __name__=="__main__":
	
	parser = argparse.ArgumentParser()
	parser.add_argument("-i", "--image", type=str,
		help="(required) name of the image that this container is dervied from", default="spartan")

	parser.add_argument("-c", "--container", type=str, default="spartan", help="(optional) name of the container")\

	parser.add_argument("-d", "--dry_run", action='store_true', help="(optional) perform a dry_run, print the command that would have been executed but don't execute it.")

	args = parser.parse_args()
	print "running docker container derived from image %s" %args.image
	source_dir=os.getcwd()

	cmd = "xhost +local:root \n"
	cmd += "nvidia-docker run -it "
	if args.container:
		cmd += " --name %(container_name)s " % {'container_name': args.container}

	cmd += " -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v %(source_dir)s:/root/spartan "  % {'source_dir': source_dir}
	cmd += " -v ~/.ssh:/root/.ssh "
	cmd += " -p 30200:30200/udp " # expose udp port
	cmd += " -p 30201:30201/udp " # expose another udp port
	
	
	cmd += args.image + "\n"
	cmd += "xhost -local:root"

	print "command = \n \n", cmd
	print ""

	# build the docker image
	if not args.dry_run:
		print "executing shell command"
		os.system(cmd)
	else:
		print "dry run, not executing command"