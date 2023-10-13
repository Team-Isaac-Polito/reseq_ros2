# If building for Jetson, pass the correct image as ARG by running
# docker buildx build -t reseq --build-arg IMAGE=dustynv/ros:humble-ros-base-l4t-r35.1.0 .
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

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# copy workspace
RUN mkdir -p /ros2_ws/src/
WORKDIR /ros2_ws
COPY reseq_interfaces src/reseq_interfaces
COPY reseq_ros2 src/reseq_ros2

RUN /ros_entrypoint.sh colcon build

CMD /bin/bash -c "source install/setup.bash && ros2 launch src/reseq_ros2/launch/reseq_launch.py"
