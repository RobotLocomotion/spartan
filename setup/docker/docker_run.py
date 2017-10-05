#!/usr/bin/env python

import argparse
import os
import getpass

if __name__=="__main__":
	user_name = getpass.getuser()
	default_image_name = user_name + '-spartan'
	parser = argparse.ArgumentParser()
	parser.add_argument("-i", "--image", type=str,
		help="(required) name of the image that this container is derived from", default=default_image_name)

	parser.add_argument("-c", "--container", type=str, default="spartan", help="(optional) name of the container")\

	parser.add_argument("-d", "--dry_run", action='store_true', help="(optional) perform a dry_run, print the command that would have been executed but don't execute it.")

	args = parser.parse_args()
	print "running docker container derived from image %s" %args.image
	source_dir=os.getcwd()

	image_name = args.image
	home_directory = '/home/' + user_name

	cmd = "xhost +local:root \n"
	cmd += "nvidia-docker run -it "
	if args.container:
		cmd += " --name %(container_name)s " % {'container_name': args.container}

	cmd += " -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v %(source_dir)s:%(home_directory)s/spartan "  % {'source_dir': source_dir, 'home_directory': home_directory}
	cmd += " -v ~/.ssh:%(home_directory)s/.ssh " % {'home_directory': home_directory}
	cmd += " --user %s " % user_name # login as current user

    # expose UDP ports
	cmd += " -p 30200:30200/udp " # expose udp ports for kuka
	cmd += " -p 30201:30201/udp " # expose udp ports for kuka
	cmd += " -p 1500:1500/udp " # expose udp ports for schunk
	cmd += " -p 1501:1501/udp " # expose udp ports for schunk

	cmd += " --privileged -v /dev/bus/usb:/dev/bus/usb " # allow usb access
	
	cmd += " --rm " # remove the image when you exit
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
