FROM ros:humble-ros-core

RUN apt update && apt install -y python3-rosdep gcc g++ make python3-colcon-common-extensions && rosdep init && rosdep update

RUN mkdir -p /ros2_ws/src/
WORKDIR /ros2_ws
COPY reseq_interfaces src/reseq_interfaces
COPY reseq_ros2 src/reseq_ros2

RUN rosdep install -iy --from-path src --rosdistro humble
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build"

CMD /bin/bash "-c" "source ./install/local_setup.bash && ros2 launch reseq_ros2 reseq_launch.py config_file:=reseq_mk2_can.yaml"