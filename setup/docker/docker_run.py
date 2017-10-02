import argparse
import os

if __name__=="__main__":
	
	parser = argparse.ArgumentParser()
	parser.add_argument("-i", "--image", type=str,
		help="(required) name of the image that this container is dervied from", required=True)

	parser.add_argument("-c", "--container", type=str, help="(optional) name of the container")\

	args = parser.parse_args()
	print "running docker container derived from image %s" %args.image
	source_dir=os.getcwd()

	cmd = "xhost +local:root \n"
	cmd += "nvidia-docker run -it "
	if args.container:
		cmd += " --name %(container_name)s " % {'container_name': args.container}

	cmd += " -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v %(source_dir)s:/root/spartan "  % {'source_dir': source_dir}

	
	
	cmd += args.image + "\n"
	cmd += "xhost -local:root"

	print "command = \n", cmd

	# build the docker image
	os.system(cmd)