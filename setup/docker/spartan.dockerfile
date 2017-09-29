FROM nvidia/cuda:8.0-devel-ubuntu16.04

RUN apt-get update

COPY ./setup/ubuntu/16.04/install_prereqs.sh /tmp/spartan_install_prereqs.sh
RUN yes "Y" | /tmp/spartan_install_prereqs.sh

COPY ./drake/setup/ubuntu/16.04/install_prereqs.sh /tmp/drake_install_prereqs.sh
RUN yes "Y" | /tmp/drake_install_prereqs.sh