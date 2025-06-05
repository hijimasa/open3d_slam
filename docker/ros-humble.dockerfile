FROM ubuntu:jammy

# Set locale
RUN locale  # check for UTF-8
RUN apt-get update && apt-get install locales -y
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 
RUN export LANG=en_US.UTF-8
RUN locale  # verify settings

#Setup Sources
RUN apt-get update && apt-get install software-properties-common -y
RUN add-apt-repository universe

RUN apt-get update && apt-get install curl -y
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
RUN export ROS_APT_SOURCE_VERSION="1.1.0" && \
    export VERSION_CODENAME="jammy" && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
RUN apt-get install /tmp/ros2-apt-source.deb -y
## Or using wget...
# RUN apt-get install wget -y
# RUN export ROS_APT_SOURCE_VERSION="1.1.0" && \
#     export VERSION_CODENAME="jammy" && \
#     wget "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
# RUN apt-get install -y ./ros2-apt-source* && rm ./ros2-apt-source*

RUN apt-get update
RUN apt-get upgrade -y

# This is an iterative dependency of ros-noetic-jsk-rviz-plugins
RUN DEBIAN_FRONTEND=noninteractive apt-get install keyboard-configuration -y 

RUN DEBIAN_FRONTEND=noninteractive apt install ros-humble-ros-base -y

####################################################################################################################################################

RUN apt-get install -y git nano

RUN apt-get update && \
    apt-get -y install sudo

# # create a user https://stackoverflow.com/questions/27701930/how-to-add-users-to-docker-container)
RUN useradd -ms /bin/bash ubuntu \
    && echo 'ubuntu:ubuntu' | chpasswd \ 
    && usermod -aG sudo ubuntu

# Small script to build as user (requires removing devel and build created by root)
WORKDIR /home/ubuntu/
RUN touch buildWithColcon.sh 
RUN echo "#!/bin/bash" >> buildWithColcon.sh
RUN echo "mkdir colcon_ws" >> buildWithColcon.sh
RUN echo "cp -r example_ws/* colcon_ws/" >> buildWithColcon.sh
RUN echo "cd colcon_ws" >> buildWithColcon.sh
RUN echo "source /opt/ros/humble/setup.bash" >> buildWithColcon.sh
RUN echo "colcon build" >> buildWithColcon.sh
RUN chmod +x buildWithColcon.sh

# dependencies
WORKDIR /home/ubuntu/example_ws/src
RUN apt update && \
    apt install -y libgoogle-glog-dev libglfw3 libglfw3-dev liblua5.2-dev \
    ros-dev-tools \
    libboost-all-dev \
    ros-humble-interactive-markers \
    software-properties-common \
    libopen3d-dev

RUN git clone https://github.com/hijimasa/open3d_slam.git -b ros2_colored

RUN sed -i 's/Open3D 0.18.0 REQUIRED/Open3D 0.14.1 REQUIRED/g' open3d_slam/ros/open3d_slam_ros/CMakeLists.txt

# acl is required to use setfacl command and give user permission to folder
WORKDIR /home/ubuntu
RUN apt install acl -y 
RUN setfacl -m u:ubuntu:rwx example_ws

WORKDIR /home/ubuntu/example_ws

# You can also build example_ws here, or use the previously built script to duplicate the workspace when running the docker (useful if you find permission issues)
RUN cd src/open3d_slam && \
    git submodule init && \
    git submodule update && \
    cd ../.. # && \
    #/bin/bash -c "source /opt/ros/humble/setup.bash; colcon build"

# login as user, commands below will be run at user level
USER ubuntu
WORKDIR /home/ubuntu/

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export EDITOR='nano -w'" >> ~/.bashrc

