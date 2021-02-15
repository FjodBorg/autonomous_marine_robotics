FROM ubuntu:bionic

# setup environment
ENV ROS_DISTRO melodic
ENV HOME "/usr/docker"
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV QT_X11_NO_MITSHM 1
ENV DISPLAY "$DISPLAY"

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list




# install ros-desktop-full and ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full=1.4.1-0* curl iputils-ping git openssh-server \ 
    python-catkin-tools software-properties-common libeigen3-dev libopencv-dev \ 
    libyaml-cpp-dev ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-tf \ 
    ros-$ROS_DISTRO-image-transport \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y 
RUN sudo add-apt-repository universe
RUN apt-get update && apt install -y build-essential curl


# install blue_rov_things
#RUN sudo apt install ros-melodic-uuv-simulator ros-melodic-gazebo-*

# setup local enviromental variabls
ARG HOME="/home/docker"
ENV HOME=$HOME
ARG repo_DIR="." 



# setup workspace
WORKDIR "$HOME/catkin_ws"
RUN mkdir -p $HOME/catkin_ws/src

# setup entrypoint
COPY "./ros_entrypoint.sh" /
ENTRYPOINT ["ros_entrypoint.sh"]

CMD ["bash"]

# launch ros package
#CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]
