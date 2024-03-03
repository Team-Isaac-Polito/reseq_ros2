# docker run --gpus all --privileged -it --rm --net=host reseq
# docker build . --target reseq-base --tag reseq-base
# docker build . --tag reseq

# At the moment an amd64 alternative with CUDA and ROS2 is yet to be chosen
ARG BASE_IMAGE=dustynv/ros:humble-ros-base-l4t-r35.1.0

FROM ${BASE_IMAGE} AS reseq-base

# Install the tools needed to compile and use librealsense
# N.B. Some of these tools are not needed to run reseq and should be removed by using multiple stages
RUN apt update && apt install -y --no-install-recommends \
        build-essential \
        python3-pip \
        git \
        libssl-dev \
        libusb-1.0-0-dev \
        libudev-dev \
        pkg-config \
        libgtk-3-dev \ 
        libglfw3-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        cmake \
        curl \
        python3-dev \
        ca-certificates \
        libusb-1.0-0 \
        udev \
        apt-transport-https

# Compile librealsense with CUDA and RSUSB
RUN git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    mkdir build && cd build && \
    cmake .. \
        -DBUILD_EXAMPLES=true \
        -DCMAKE_BUILD_TYPE=release \
        -DFORCE_RSUSB_BACKEND=true \
        -DBUILD_WITH_CUDA=true \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_PYTHON_BINDINGS=bool:true \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DPYTHON_INSTALL_DIR=$(python3 -c 'import sys; print(f"/usr/lib/python{sys.version_info.major}.{sys.version_info.minor}/dist-packages")') && \
    make -j$(($(nproc)-1)) && \
    make install && \
    cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/ && \
    cp ../config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/ && \
    cd / && \
    rm -rf librealsense 

ENV LD_LIBRARY_PATH /usr/local/lib:$LD_LIBRARY_PATH

# Docker complains about missing UDEV rules but being root, it is not a problem
# RUN service udev start && udevadm control --reload-rules && udevadm trigger

# Setup ROS2 environment to compile ROS packages
RUN pip install \
        colcon-common-extensions \
        colcon-mixin 

RUN colcon mixin add default \
        https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
        https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

RUN apt update && apt install python3-rosdep -y 

# Compile realsense-ros
RUN source /ros_entrypoint.sh && \
    mkdir -p /realsense-ros/src && \
    cd /realsense-ros && \
    rosinstall_generator --rosdistro ${ROS_DISTRO} \
        xacro \
        diagnostic_updater \
        sensor_msgs_py \
        launch_pytest \
        realsense2_camera \
        realsense2_camera_msgs \
        realsense2_description \
        > realsense.rosinstall && \
    vcs import src < realsense.rosinstall && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    sed -i "\$i ros_source_env /realsense-ros/install/local_setup.bash \n" /ros_entrypoint.sh

ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp

RUN pip install python-can

FROM reseq-base AS reseq

RUN mkdir -p /ros2_ws/src/
WORKDIR /ros2_ws
COPY reseq_interfaces src/reseq_interfaces
COPY reseq_ros2 src/reseq_ros2

RUN source /ros_entrypoint.sh && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build && \
    sed -i "\$i ros_source_env /ros2_ws/install/setup.bash \n" /ros_entrypoint.sh

CMD /bin/bash -c "source /ros_entrypoint.sh && ros2 launch reseq_ros2 reseq_launch.py"
