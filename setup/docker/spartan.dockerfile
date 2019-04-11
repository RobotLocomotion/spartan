FROM nvidia/cuda:10.0-devel-ubuntu16.04

ARG USER_NAME
ARG USER_PASSWORD
ARG USER_ID
ARG USER_GID

RUN apt-get update
RUN apt install sudo
RUN useradd -ms /bin/bash $USER_NAME
RUN usermod -aG sudo $USER_NAME
RUN yes $USER_PASSWORD | passwd $USER_NAME

# set uid and gid to match those outside the container
RUN usermod -u $USER_ID $USER_NAME 
RUN groupmod -g $USER_GID $USER_NAME


# working directory is /home/$USER_NAME
WORKDIR /home/$USER_NAME
ENV USER_HOME_DIR=/home/$USER_NAME

# require no sudo pw in docker
# RUN echo $USER_PASSWORD | sudo -S bash -c 'echo "'$USER_NAME' ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/docker-user' && printf "\n"

COPY ./setup/docker/install_dependencies.sh /tmp/install_dependencies.sh
RUN yes "Y" | /tmp/install_dependencies.sh

COPY ./setup/ubuntu/16.04/install_prereqs.sh /tmp/spartan_install_prereqs.sh
RUN yes "Y" | /tmp/spartan_install_prereqs.sh

COPY ./drake/setup/ubuntu /tmp/drake_setup
RUN yes "Y" | /tmp/drake_setup/install_prereqs.sh

# Hack needed to deal with bazel issue, see https://github.com/bazelbuild/bazel/issues/4483
#COPY ./setup/docker/install_dependencies_drake.sh /tmp/drake_install_prereqs.sh
#RUN yes "Y" | /tmp/drake_install_prereqs.sh

COPY ./director/distro/travis/install_deps.sh /tmp/director_travis_install_prereqs.sh
RUN yes "Y" | TRAVIS_OS_NAME=linux /tmp/director_travis_install_prereqs.sh

# install handical
COPY ./setup/docker/install_handical_dependencies.sh /tmp/install_handical_dependencies.sh
RUN yes "Y" | /tmp/install_handical_dependencies.sh


# set the terminator inside the docker container to be a different color
RUN mkdir -p .config/terminator
COPY ./setup/docker/terminator_config .config/terminator/config
RUN chown $USER_NAME:$USER_NAME -R .config

# PDC
COPY ./src/catkin_projects/pytorch-dense-correspondence-private/docker/install_dependencies.sh /tmp/pdc_install_dependencies.sh
RUN yes "Y" | /tmp/pdc_install_dependencies.sh

COPY ./src/catkin_projects/pytorch-dense-correspondence-private/docker/install_more.sh /tmp/pdc_install_more.sh
RUN yes "Y" | /tmp/pdc_install_more.sh

COPY ./src/catkin_projects/pytorch-dense-correspondence-private/docker/install_poser.sh /tmp/pdc_install_poser.sh
RUN yes "Y" | /tmp/pdc_install_poser.sh
# PDC


# setup bazel bashrc
# RUN echo "startup --output_base=/home/$USER_NAME/.spartan-build" >> .bazelrc
RUN echo "startup --output_base=/home/$USER_NAME/.spartan-build" >> /etc/bazel.bazelrc

# change ownership of everything to our user
RUN cd /home/$USER_NAME && chown $USER_NAME:$USER_NAME -R .


###### COMMENT OUT THIS BLOCK IF USING NVIDIA-DOCKER1 ###########
# needed to get OpenGL running inside the docker
# https://github.com/machinekoder/nvidia-opengl-docker

# optional, if the default user is not "root", you might need to switch to root here and at the end of the script to the original user again.
# e.g.
# USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config \
        libxau-dev \
        libxdmcp-dev \
        libxcb1-dev \
        libxext-dev \
        libx11-dev && \
    rm -rf /var/lib/apt/lists/*

# replace with other Ubuntu version if desired
# see: https://hub.docker.com/r/nvidia/opengl/
# e.g. nvidia/opengl:1.1-glvnd-runtime-ubuntu16.04)
COPY --from=machinekoder/nvidia-opengl-docker:1.1-glvnd-runtime-stretch \
  /usr/local/lib/x86_64-linux-gnu \
  /usr/local/lib/x86_64-linux-gnu

# replace with other Ubuntu version if desired
# see: https://hub.docker.com/r/nvidia/opengl/
# e.g. nvidia/opengl:1.1-glvnd-runtime-ubuntu16.04
COPY --from=machinekoder/nvidia-opengl-docker:1.1-glvnd-runtime-stretch \
  /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json \
  /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json

RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    ldconfig && \
    echo '/usr/local/$LIB/libGL.so.1' >> /etc/ld.so.preload && \
    echo '/usr/local/$LIB/libEGL.so.1' >> /etc/ld.so.preload

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# USER original_user
###### END BLOCK ###########

RUN pip install https://download.pytorch.org/whl/cu100/torch-1.0.1.post2-cp27-cp27mu-linux_x86_64.whl
RUN pip install torchvision

ENTRYPOINT bash -c "source ~/spartan/setup/docker/entrypoint.sh && source ~/spartan/src/catkin_projects/pytorch-dense-correspondence-private/docker/entrypoint.sh && /bin/bash"


