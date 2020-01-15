FROM ros:melodic-ros-core-bionic
WORKDIR /root/catkin_ws
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && \
    apt install -y iputils-ping net-tools binutils-arm-none-eabi git cmake libusb-1.0 \
    python-sphinx python-breathe python-wstool python-catkin-tools \
    ros-melodic-rosserial-msgs ros-melodic-rosserial-python && \
    wstool init src && \
    cd ../ && \
    git clone https://github.com/texane/stlink.git && \
    cd stlink && \
    make && \
    cd build/Release/ && \
    make install && \
    ldconfig && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* 
