import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/reinoxe/ros2_ws/install/global_coordinate_simple_mover'
