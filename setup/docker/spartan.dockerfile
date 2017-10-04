FROM nvidia/cuda:8.0-devel-ubuntu16.04

RUN apt-get update

COPY ./setup/docker/install_dependencies.sh /tmp/install_dependencies.sh
RUN yes "Y" | /tmp/install_dependencies.sh

COPY ./setup/ubuntu/16.04/install_prereqs.sh /tmp/spartan_install_prereqs.sh
RUN yes "Y" | /tmp/spartan_install_prereqs.sh

COPY ./drake/setup/ubuntu/16.04/install_prereqs.sh /tmp/drake_install_prereqs.sh
RUN yes "Y" | /tmp/drake_install_prereqs.sh

RUN mkdir -p /root/.config/terminator
COPY ./setup/docker/terminator_config /root/.config/terminator/config

ENTRYPOINT bash -c "source /root/spartan/setup/docker/entrypoint.sh && /bin/bash"