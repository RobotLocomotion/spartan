import argparse
import os

if __name__=="__main__":
	print "building docker container . . . "
	parser = argparse.ArgumentParser()
	parser.add_argument("-i", "--image", type=str,
		help="name for the newly created docker image")

	args = parser.parse_args()
	print "building docker image named ", args.image_name
	command = "docker build -t " + args.image_name + " -f setup/docker/spartan.dockerfile ."
	print "command = ", command

	# build the docker image
	os.system(command)