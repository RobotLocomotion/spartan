# Spartan inside Docker

## Disclaimer (not supported for public use)

This is a project repository for robotics research and applications using
[Drake](https://drake.mit.edu/) and [Director](https://github.com/RobotLocomotion/director). This repository is meant as an internal tool for the Robot Locomotion Group. **It contains private submodules and is not supported for external use**. 

## Quickstart

The following is all of the steps to build spartan with docker from a fresh Ubuntu installation:

1) Install [Docker for Ubuntu](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)
  - Make sure to `sudo usermod -aG docker your-user` and then not run below docker scripts as `sudo`. After applying this change you made need to log out and then log back in for the changes to take effect.
2) Install [`nvidia-docker`](https://github.com/NVIDIA/nvidia-docker). We are now using `nvidia-docker2`.

and then restart your machine.

NOTE: it would be nice to do a minimal nvidia-docker test.  The one we used to run here doesn't seem to work with nvidia-docker2.


3) Clone, setup, and build Spartan: You need to have ssh keys setup to clone the submodules. Make sure that these ssh keys don't have a password, otherwise it will not work.
```
git clone git@github.com:RobotLocomotion/spartan.git
cd spartan
git submodule sync
git submodule update --init
./setup/docker/docker_build.py
./setup/docker/docker_run.py
mkdir build && cd build && use_ros && cmake .. && use_spartan && make -j8
```
Below is explained additional options and details of the above.

## Building Docker Image
After cloning spartan and setting up submodules, from the spartan root directory run

```./setup/docker/docker_build.py```

to build a docker image. The default image name is `<username>-spartan` where `<username>` is your username on the host machine. Optionally you can pass the option `-i <image_name>` to specify the name of the image. Use `-h` to see a full list of command line options. Note you can also rename the image after building (see Docker Cheatsheet below).

This is a basic 16.04 environment with the necessary dependencies for `spartan` installed.

## Running a Docker Container
To create container derived from the image that was just build run (from the `spartan` root directory)

```
./setup/docker/docker_run.py
```

By default this will create a container named `<username>-spartan` derived from an image named `<username>-spartan`. It also creates a user named `<username>` inside the docker container. By default this username will match the current username on the host machine. This is important for SSH keys used in cloning private github repos to work properly. The default password for your user inside the docker container is `password`. User the `-p` flag to set a different password.

This container mounts your current
spartan directory at `/home/<username>/spartan`. You can use the optional command line arguments `-i <image_name>` to derive a container from a specific image, and `-c <container_name>` to name the container. Use `-h` to see a full list of command line arguments. By default the container will be deleted upon exiting the container (due to the `--rm` flag), but both the source code and the compiled binaries live outside the container.

See [here](https://docs.docker.com/engine/reference/commandline/start/) for more documentation on the `docker run` command.

## Building Spartan inside the Docker container
```
cd /root/spartan
mkdir build && cd build
use_ros
cmake ..
use_spartan
make -j
```

## Environment setup


After you configure the build you will find a file named ``setup_environment.sh``
inside the build folder. The alias `use_spartan` in your docker environment sources this file. You should read the contents of ``setup_environment.sh`` to see what it does.
In addition to modifying your PATH and other variables, it also defines some
useful aliases for developers.

## Docker Cheatsheet

**Handling images**
- `docker images` - lists all docker images on machine, including REPOSITORY, TAG, IMAGE_ID, when created, size
- `docker tag IMAGE_ID NEW_NAME` - creates a new REPOSITORY:TAG for an IMAGE_ID
- `docker rmi REPOSITORY:TAG` - removes this tag for an image
- `docker tag IMAGE_ID my-spartan && docker rmi spartan` -- example to combine above two commands to rename an image ID
- clean up stale artifacts `docker system prune`.

**Handling containers**
- `docker ps -a` - lists all containers on machine
- `docker rm CONTAINER_ID` - removes container id 

**Cleaning Up**

See [this](https://www.digitalocean.com/community/tutorials/how-to-remove-docker-images-containers-and-volumes) link.
- `docker system prune` prune dangling images, containes
