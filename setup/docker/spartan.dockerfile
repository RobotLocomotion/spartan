FROM nvidia/cuda:8.0-devel-ubuntu16.04

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


WORKDIR /home/$USER_NAME
# require no sudo pw in docker
# RUN echo $USER_PASSWORD | sudo -S bash -c 'echo "'$USER_NAME' ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/docker-user' && printf "\n"

COPY ./setup/docker/install_dependencies.sh /tmp/install_dependencies.sh
RUN yes "Y" | /tmp/install_dependencies.sh

COPY ./setup/ubuntu/16.04/install_prereqs.sh /tmp/spartan_install_prereqs.sh
RUN yes "Y" | /tmp/spartan_install_prereqs.sh

COPY ./drake/setup/ubuntu/16.04/install_prereqs.sh /tmp/drake_install_prereqs.sh
RUN yes "Y" | /tmp/drake_install_prereqs.sh

COPY ./director/distro/travis/install_deps.sh /tmp/director_travis_install_prereqs.sh
RUN yes "Y" | TRAVIS_OS_NAME=linux /tmp/director_travis_install_prereqs.sh

# set the terminator inside the docker container to be a different color
RUN mkdir -p .config/terminator
COPY ./setup/docker/terminator_config .config/terminator/config
RUN chown $USER_NAME:$USER_NAME -R .config

# change ownership of everything to our user
RUN cd $WORKDIR && chown $USER_NAME:$USER_NAME -R .

ENTRYPOINT bash -c "source ~/spartan/setup/docker/entrypoint.sh && /bin/bash"


