# Building Docker Image
From the spartan root directory run

```python ./setup/docker/docker_build.py```

to build a docker image. Optionally you can pass the option `-i <image_name>` to specify the name of the image. Use `-h` to see a full list of command line options. This is a basic 16.04 environment with the necessary dependencies for `spartan` installed.

# Running a Docker Container
To create container derived from the image that was just build run (from the `spartan` root directory)

```
python ./setup/docker/docker_build.py
```

By default this will create a container named `spartan` derived from an image named `spartan`. This container mounts your current
spartan directory at `/root/spartan`. You can use the optional command line arguments `-i <image_name>` to derive a container from a specific image, and `-c <container_name>` to name the container. Use `-h` to see a full list of command line arguments. To run an already existing container just do

```
docker start -i <container_name>
```

See [here](https://docs.docker.com/engine/reference/commandline/start/) for more documentation on the `docker run` command.

# Building Spartan inside the Docker container
```
cd /root/spartan
mkdir build && cd build
cmake ..
make -j
```
