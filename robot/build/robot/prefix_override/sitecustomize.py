import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/stepa/ros2_ws/diplom-robot/robot/install/robot'
