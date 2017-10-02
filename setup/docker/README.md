# Building Docker Image
From the spartan root directory run

```python ./setup/docker/docker_build.py -i <image_name>```

to build a docker image with name
`<image_name>`. This is a basic 16.04 environment with the necessary dependencies for `spartan` installed.

# Running a Docker Container
To create container derived from the image that was just build run (from the `spartan` root directory)

```
python ./setup/docker/docker_build.py -i <image_name> -c <container_name>
```

note that `<image_name>` should match the name you gave to the image wheny you created it in the previous step. This
container mounts your current spartan directory at `/root/spartan`. To run an already existing container just do

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
