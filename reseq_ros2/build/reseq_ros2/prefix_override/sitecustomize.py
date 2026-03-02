import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/salvatore/ros2_ws/src/reseq_ros2/install/reseq_ros2'
