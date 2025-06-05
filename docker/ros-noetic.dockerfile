FROM ros:noetic-ros-base
# https://github.com/osrf/docker_images/issues/814

RUN apt-get update \
    && apt-get install -y git nano

# create a user https://stackoverflow.com/questions/27701930/how-to-add-users-to-docker-container)
RUN useradd -ms /bin/bash ubuntu \
    && echo 'ubuntu:ubuntu' | chpasswd \ 
    && usermod -aG sudo ubuntu

# Small script to build as user (requires removing devel and build created by root)
WORKDIR /home/ubuntu/
RUN touch buildWithCatkin.sh 
RUN echo "#!/bin/bash" >> buildWithCatkin.sh
RUN echo "mkdir catkin_ws" >> buildWithCatkin.sh
RUN echo "cp -r example_ws/* catkin_ws/" >> buildWithCatkin.sh
RUN echo "cd catkin_ws" >> buildWithCatkin.sh
RUN echo "source /opt/ros/noetic/setup.bash" >> buildWithCatkin.sh
RUN echo "catkin_make" >> buildWithCatkin.sh
RUN chmod +x buildWithCatkin.sh

# This is an iterative dependency of ros-noetic-jsk-rviz-plugins
RUN DEBIAN_FRONTEND=noninteractive apt-get install keyboard-configuration -y 

RUN apt update && \
    apt install libgoogle-glog-dev libglfw3 libglfw3-dev liblua5.2-dev -y 

RUN apt update && apt install ros-noetic-jsk-rviz-plugins -y

WORKDIR /home/ubuntu/example_ws/src
RUN apt-get install -y \
    software-properties-common \
    libboost-all-dev \
    ros-noetic-eigen-conversions \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros


RUN add-apt-repository ppa:roehling/open3d 
RUN apt update 
RUN apt install libopen3d-dev -y
RUN git clone https://github.com/leggedrobotics/open3d_slam.git

# acl is required to use setfacl command and give user permission to folder
WORKDIR /home/ubuntu
RUN apt install acl -y 
RUN setfacl -m u:ubuntu:rwx example_ws

# login as user, commands below will be run at user level
USER ubuntu
WORKDIR /home/ubuntu/

# Source ROS and set nano as default editor
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "export EDITOR='nano -w'" >> ~/.bashrc


