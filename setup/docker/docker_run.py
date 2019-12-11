#!/usr/bin/env python
from __future__ import print_function

import argparse
import os
import getpass
import yaml
import socket

user_name = getpass.getuser()
host_name = socket.gethostname()
DATA_DIRECTORY_ON_HOST = '/home/' + user_name + '/data/spartan'


CONFIG_FILE = os.path.join(os.getcwd(), 'setup/docker/docker_run_config.yaml')

if __name__ == "__main__":
    user_name = getpass.getuser()
    default_image_name = user_name + '-spartan'
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--image", type=str,
                        help="(required) name of the image that this container is derived from",
                        default=default_image_name)

    parser.add_argument("-c", "--container", type=str, default="spartan", help="(optional) name of the container")

    parser.add_argument("-d", "--dry_run", action='store_true',
                        help="(optional) perform a dry_run, print the command that would have been executed but don't execute it.")

    parser.add_argument("-e", "--entrypoint", type=str, default="", help="(optional) thing to run in container")

    parser.add_argument("-p", "--passthrough", type=str, default="",
                        help="(optional) extra string that will be tacked onto the docker run command, allows you to pass extra options. Make sure to put this in quotes and leave a space before the first character")

    parser.add_argument("-nodudp", "--no_udp", action='store_true', help="(optional) don't expose the udp ports")

    parser.add_argument("--no_nvidia", action='store_true',
                        help="(optional) use docker rather than nvidia-docker (for CI)")

    parser.add_argument("-hn", "--host_network", action='store_true', help="(optional) whether to use the host's network")

    args = parser.parse_args()
    print("running docker container derived from image %s" % args.image)
    source_dir = os.getcwd()

    image_name = args.image
    home_directory = '/home/' + user_name
    spartan_source_dir = os.path.join(home_directory, 'spartan')

    cmd = "xhost +local:root \n"
    if args.no_nvidia:
        cmd += "docker run "
    else:
        cmd += "nvidia-docker run "
    if args.container:
        cmd += " --name %(container_name)s " % {'container_name': args.container}

    cmd += " -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw "  # enable graphics
    cmd += " -v %(source_dir)s:%(home_directory)s/spartan " \
           % {'source_dir': source_dir, 'home_directory': home_directory}  # mount source
    cmd += " -v ~/fullstack/src/log_robot:%(home_directory)s/log_robot " % {'home_directory': home_directory}  # mount log data
    cmd += " -v ~/.ssh:%(home_directory)s/.ssh " % {'home_directory': home_directory}  # mount ssh keys

    # Make Bazel artifact dir if it doesn't exist
    bazel_artifact_dir = "~/.spartan-docker/%(image_name)s-build" % {'image_name': image_name}
    mkdir_cmd = "mkdir -p %s" % bazel_artifact_dir
    print("command = ", mkdir_cmd)
    os.system(mkdir_cmd)

    cmd += " -v %(bazel_artifact_dir)s:%(home_directory)s/.spartan-build " \
           % {'bazel_artifact_dir': bazel_artifact_dir,
              'home_directory': home_directory}  # mount bazel build artifact dirs
    cmd += " --user %s " % user_name  # login as current user

    # mount the data volume
    data_directory_host_machine = None
    sandbox_directory_host_machine = None

    config_yaml = yaml.load(open(CONFIG_FILE))
    if host_name in config_yaml:
        if user_name in config_yaml[host_name]:
            data_directory_host_machine = config_yaml[host_name][user_name]["path_to_data_directory"]

            if "path_to_sandbox_directory" in config_yaml[host_name][user_name]:
                sandbox_directory_host_machine = config_yaml[host_name][user_name]["path_to_sandbox_directory"]

    if data_directory_host_machine is None:
        data_directory_host_machine = '/home/' + user_name + '/data'
        os.system("mkdir -p " + data_directory_host_machine)

    cmd += " -v %s:%s/data " % (data_directory_host_machine, home_directory)

    if sandbox_directory_host_machine is None:
        sandbox_directory_host_machine = os.path.join('~', 'sandbox')
        if not os.path.exists(sandbox_directory_host_machine):
            os.makedirs(sandbox_directory_host_machine)

    cmd += " -v %s:%s/sandbox " % (sandbox_directory_host_machine, home_directory)

    # expose UDP ports
    if not args.no_udp:
        cmd += " -p 30200:30200/udp "  # expose udp ports for kuka
        cmd += " -p 30201:30201/udp "  # expose udp ports for kuka
        cmd += " -p 1500:1500/udp "  # expose udp ports for schunk
        cmd += " -p 1501:1501/udp "  # expose udp ports for schunk

    if args.host_network:
        cmd += " --network=host "

    cmd += " " + args.passthrough + " "

    cmd += " --privileged -v /dev/bus/usb:/dev/bus/usb "  # allow usb access
    cmd += " --privileged -v /dev/hydra:/dev/hydra "  # allow razer hydra alias

    cmd += " --rm "  # remove the image when you exit

    # allow setting chrt priority by regular user inside container???
    cmd += " --ulimit rtprio=30 "

    if args.entrypoint and args.entrypoint != "":
        cmd += "--entrypoint=\"%(entrypoint)s\" " % {"entrypoint": args.entrypoint}
    else:
        cmd += "-it "
    cmd += image_name
    cmd_endxhost = "xhost -local:root"

    print("command = \n \n", cmd, "\n", cmd_endxhost)
    print("")

    # build the docker image
    if not args.dry_run:
        print("executing shell command")
        code = os.system(cmd)
        print("Executed with code ", code)
        os.system(cmd_endxhost)
        # Squash return code to 0/1, as
        # Docker's very large return codes
        # were tricking Jenkins' failure
        # detection
        exit(code != 0)
    else:
        print("dry run, not executing command")
        exit(0)
