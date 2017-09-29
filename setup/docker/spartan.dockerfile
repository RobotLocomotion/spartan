FROM nvidia/cuda:8.0-devel-ubuntu16.04
RUN mkdir /spartan
COPY . /spartan
RUN  ls \
	  && apt-get update \ 
      && yes "Y" | /spartan/setup/ubuntu/16.04/install_prereqs.sh \
      && yes "Y" | /spartan/drake/setup/ubuntu/16.04/install_prereqs.sh \ 
      && rm -rf /var/lib/apt/lists/* \
      && apt-get clean all