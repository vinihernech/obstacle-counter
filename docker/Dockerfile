FROM osrf/ros:humble-desktop-full

SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update \
  && apt-get install -y apt-utils \
  && apt-get -y install nano \
  && apt-get install -y python3-pip \
  && pip3 install scikit-learn \
  && apt-get -y install ros-humble-navigation2 ros-humble-nav2-bringup \
  && apt-get -y install ros-humble-turtlebot3* 

RUN apt-get update && apt-get install --no-install-recommends -y \
    && apt-get -y install build-essential \
    && apt-get -y install git \
    && apt-get -y install python3-colcon-common-extensions \
    && apt-get -y install python3-colcon-mixin \
    && apt-get -y install python3-rosdep \
    && apt-get -y install python3-vcstool 
   

ENV ROS2_WS /opt/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR /opt/ros2_ws/src
COPY . .
WORKDIR $ROS2_WS
RUN colcon build --cmake-args -DSECURITY=ON --no-warn-unused-cli --symlink-install
                  
RUN source /opt/ros/humble/setup.bash 
RUN source /opt/ros2_ws/install/local_setup.bash
