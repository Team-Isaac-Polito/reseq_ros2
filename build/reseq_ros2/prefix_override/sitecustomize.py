import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/salvatore/ros2_ws/src/install/reseq_ros2'
