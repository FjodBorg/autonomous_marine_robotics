FROM ubuntu:bionic

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

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO melodic



# install ros-desktop-full and ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full curl iputils-ping git openssh-server \ 
    python-catkin-tools software-properties-common libeigen3-dev libopencv-dev \ 
    libyaml-cpp-dev ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-tf \ 
    ros-$ROS_DISTRO-image-transport \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y 
RUN sudo add-apt-repository universe
RUN apt-get update && apt install -y build-essential curl



#change shell
SHELL ["/bin/bash", "-c"]

# add user
ARG USER="docker"
RUN useradd -ms /bin/bash $USER && usermod -aG sudo $USER && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USER

# setup local enviromental variabls
ARG DISPLAY=":0"
ENV HOME=/home/$USER
ENV DISPLAY=$DISPLAY
ENV QT_X11_NO_MITSHM=1

WORKDIR $HOME

# environment setup
RUN mkdir -p $HOME/catkin_ws/src

# setup workspace
WORKDIR "$HOME/catkin_ws"
RUN source /opt/ros/melodic/setup.sh

# packages i like
RUN sudo apt-get install -y bash-completion gedit

# install blue_rov_things
RUN cd $HOME/catkin_ws/src && git clone https://github.com/tu-darmstadt-ros-pkg/hector_localization \
&& git clone -b kinetic-devel https://github.com/ros-teleop/teleop_tools \
&& git clone https://github.com/FletcherFT/bluerov2.git

RUN sudo apt install -y ros-melodic-uuv-simulator ros-melodic-gazebo-* ros-melodic-geographic-msgs geographiclib-tools
RUN sudo apt-get install -y python-catkin-tools python-rosinstall-generator python-wstool python-rosdep

RUN source /opt/ros/melodic/setup.sh && sudo rosdep init && rosdep update --as-root apt:false

RUN source /opt/ros/$ROS_DISTRO/setup.sh && cd $HOME/catkin_ws && catkin init && wstool init src \
&& rosinstall_generator --rosdistro $ROS_DISTRO mavlink | tee /tmp/mavros.rosinstall \
&& rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall \
&& wstool merge -t src /tmp/mavros.rosinstall && wstool update -t src -j4 


RUN sudo apt install ros-$ROS_DISTRO-ros-base ros-$ROS_DISTRO-rosbash --reinstall
RUN sudo apt remove jre -y
RUN sudo apt install openjdk-11-jdk  -y 
RUN mkdir $HOME/repos/ && cd $HOME/repos/ \
&& source $HOME/.bashrc \
&& git clone https://github.com/LSTS/neptus.git \
&& cd neptus \
&& git checkout 38c7f41a9885c6b059f79b38861edb4b7b67511b \
./gradlew
RUN cd $HOME/catkin_ws/src/bluerov2 && cp bluerov2_neptus/vehicle-defs/00-bluerov2-1.nvcl $HOME/repos/neptus/vehicles-defs/. 

RUN sudo apt install python-pip -y && python -m pip install future
RUN sudo apt install libxml2-dev libxslt-dev python-dev python-lxml libgeographic-dev ros-$ROS_DISTRO-geographic-msgs -y 
RUN cd $HOME/catkin_ws/src && git clone -b noetic-devel https://github.com/FletcherFT/imc_ros_bridge.git 

RUN sudo apt install -y ros-melodic-geographic-info ros-melodic-mavros ros-melodic-robot-localization

RUN cd $HOME/catkin_ws && source /opt/ros/$ROS_DISTRO/setup.sh && catkin build bluerov2_state_estimation

RUN cd $HOME/catkin_ws && source /opt/ros/$ROS_DISTRO/setup.sh && catkin build


RUN source /opt/ros/$ROS_DISTRO/setup.sh && cd $HOME/catkin_ws  && rosdep install --from-paths src --ignore-src -y \
&& sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh \
&& catkin build

RUN sudo apt-get install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl unzip

RUN mkdir $HOME/apps/ && cd $HOME/apps \
&& wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage \
&& chmod +x ./QGroundControl.AppImage # Tested with version 4.1.1

RUN sudo apt-get install -y fuse libpulse-mainloop-glib0 

RUN mkdir -p $HOME/repos/gcc-stm && cd $HOME/repos/gcc-stm \
&& wget https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2 \
&& tar -xjvf gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2 \
&& export PATH=$PATH:pwd/gcc-arm-none-eabi-6-2017-q2-update/bin \
&& sudo usermod -a -G dialout $USER

RUN sudo apt-get install -y python-dev python-opencv python-wxgtk4.0 python-pip python-matplotlib python-lxml python-pygame \
&& pip install PyYAML mavproxy --user \
&& echo "export PATH=$PATH:$HOME/.local/bin" >> $HOME/.bashrc

RUN cd $HOME/catkin_ws/src \
&& git clone https://github.com/ArduPilot/ardupilot.git \
&& cd ardupilot && git submodule update --init --recursive \
&& ./waf configure --board sitl && ./waf sub 

RUN cd $HOME/catkin_ws/src/ardupilot \
&& ./Tools/environment_install/install-prereqs-ubuntu.sh -y \
&& source ~/.profile

# SOURCING
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bashrc 
RUN echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc 
RUN echo $'if ! shopt -oq posix; then \n\
  if [ -f /usr/share/bash-completion/bash_completion ]; then \n\
    . /usr/share/bash-completion/bash_completion \n\
  elif [ -f /etc/bash_completion ]; then \n\
    . /etc/bash_completion \n\
  fi \n\
fi ' >>  $HOME/.bashrc 

RUN sudo apt install ros-melodic-geodesy psmisc -y 

RUN cd $HOME/repos/neptus/ && source $HOME/.bashrc && ./gradlew
RUN pip install casadi scipy

RUN python -m pip install --upgrade pip && python -m pip install --upgrade setuptools && python -m pip install scipy --force-reinstall

# with nvidia/cuda drivers
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
    
# setup entrypoint
COPY "./entrypoint_setup.sh" /

ENTRYPOINT ["/entrypoint_setup.sh"]
CMD ["bash"]

# launch ros package
#CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]
