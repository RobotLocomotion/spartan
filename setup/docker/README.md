# Spartan inside Docker

## Quickstart

The following is all of the steps to build spartan with docker from a fresh Ubuntu installation:

1) Install [Docker for Ubuntu](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)
2) Install [`nvidia-docker`](https://github.com/NVIDIA/nvidia-docker)
3) Clone, setup, and build Spartan:
```
git clone git@github.com:RobotLocomotion/spartan.git
cd spartan
git submodule init
git submodule update
./setup/docker/docker_build.py
./setup/docker/docker_run.py
cd root/spartan && mkdir build && cd build && cmake .. && make -j8
```

Below is explained additional options and details of the above.

## Building Docker Image
After cloning spartan and setting up submodules, from the spartan root directory run

```./setup/docker/docker_build.py```

to build a docker image. Optionally you can pass the option `-i <image_name>` to specify the name of the image. Use `-h` to see a full list of command line options. Note you can also rename the image after building (see Docker Cheatsheet below).

This is a basic 16.04 environment with the necessary dependencies for `spartan` installed.

## Running a Docker Container
To create container derived from the image that was just build run (from the `spartan` root directory)

```
./setup/docker/docker_run.py
```

By default this will create a container named `spartan` derived from an image named `spartan`. This container mounts your current
spartan directory at `/root/spartan`. You can use the optional command line arguments `-i <image_name>` to derive a container from a specific image, and `-c <container_name>` to name the container. Use `-h` to see a full list of command line arguments. To run an already existing container just do

```
docker start -i <container_name>
```

See [here](https://docs.docker.com/engine/reference/commandline/start/) for more documentation on the `docker run` command.

## Building Spartan inside the Docker container
```
cd /root/spartan
mkdir build && cd build
cmake ..
make -j
```

## Docker Cheatsheet

Handling images
- `docker images` - lists all docker images on machine, including REPOSITORY, TAG, IMAGE_ID, when created, size
- `docker tag IMAGE_ID NEW_NAME` - creates a new REPOSITORY:TAG for an IMAGE_ID
- `docker rmi REPOSITORY:TAG` - removes this tag for an image
- `docker tag IMAGE_ID my-spartan && docker rmi spartan` -- example to combine above two commands to rename an image ID

Handling containers
- `docker ps -a` - lists all containers on machine
- `docker rm CONTAINER_ID` - removes container id 
