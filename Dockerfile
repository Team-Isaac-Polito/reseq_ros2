# If building for Jetson, pass the correct image as ARG by running
# docker buildx build -t reseq --build-arg IMAGE=dustynv/ros:humble-ros-base-l4t-r32.7.1
ARG IMAGE=ros:humble-ros-core-jammy
FROM $IMAGE

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-can \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
#RUN apt-get update && apt-get install -y --no-install-recommends \
#    # ros-humble-ros-base=0.
#    ros-humble-demo-nodes-cpp \
#    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update #&& rosdep install -y \
    && rm -rf /var/lib/apt/lists/*

# copy workspace
COPY docker/entrypoint.sh /
RUN mkdir -p /ros2_ws/src/
WORKDIR /ros2_ws
COPY reseq_interfaces src/reseq_interfaces
COPY reseq_ros2 src/reseq_ros2

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build

CMD /bin/bash -c "source install/setup.bash && ros2 run reseq_ros2 communication"
