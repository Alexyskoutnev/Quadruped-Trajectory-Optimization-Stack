# MAINTAINER Alexy Skoutnev
# LABEL version="1.0.1"
# LABEL description="Dockerfile to build towr and run simulator and towr together"
FROM ubuntu:18.04
RUN DEBIAN_FRONTEND=noninteractive
WORKDIR /root
RUN echo "Trying to install dependecies to Docker image"
RUN apt-get -y update
#install dependecies
RUN apt-get -y install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev curl python3 python3-pip cmake lsb-core vim
#install Towr dependecies
RUN apt-get -y install libeigen3-dev coinor-libipopt-dev libncurses5-dev xterm
RUN mkdir -p ./thirdparty
WORKDIR /root/thirdparty
#installing Ipopt
RUN git clone https://github.com/coin-or/Ipopt.git
WORKDIR /root/thirdparty/Ipopt
RUN mkdir -p build
WORKDIR /root/thirdparty/Ipopt/build
RUN ../configure
RUN make -j4
WORKDIR /root/thirdparty
#installing ifopt
RUN git clone https://github.com/ethz-adrl/ifopt.git
WORKDIR /root/thirdparty/ifopt
RUN mkdir -p build
WORKDIR /root/thirdparty/ifopt/build
RUN cmake ..
RUN make -j4
RUN make install
WORKDIR /root/
#installing ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-melodic.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update
RUN DEBIAN_FRONTEND=noninteractive apt-get -y install ros-melodic-desktop-full
WORKDIR /
RUN bash -c "source /opt/ros/melodic/setup.bash >> /root/.bashrc"
RUN bash -c "source /root/.bashrc"
#installing towr
RUN apt-get install -y ros-melodic-xpp
WORKDIR /root/
RUN mkdir -p catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/Alexyskoutnev/towr_solo12.git towr_solo12
WORKDIR /root/catkin_ws/src/towr_solo12/towr
RUN mkdir build
WORKDIR /root/catkin_ws/src/towr_solo12/towr/build
RUN cmake -S ../ -DCMAKE_BUILD_TYPE=Release
RUN make -j4
RUN make install
WORKDIR /root/catkin_ws/src/towr_solo12/towr/build