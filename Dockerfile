FROM ghcr.io/europeanroverchallenge/erc-remote-image-base:latest

# Install additional packages
RUN apt-get update && apt-get -y upgrade && apt-get -y install \
  tmux \
  && rm -rf /var/lib/apt/lists/*

# Copy packages and build the workspace
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /catkin_ws
COPY src ./src
RUN apt-get update \
  && rosdep update \
  && rosdep install --from-paths src -iy \
  && apt-get install libgeographic-dev ros-noetic-geographic-msgs \
  && rm -rf /var/lib/apt/lists/* 
RUN catkin config --extend /opt/ros/noetic && catkin build --no-status

# Automatically source the workspace when starting a bash session
RUN echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc
