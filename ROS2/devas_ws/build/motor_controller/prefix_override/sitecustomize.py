import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tac/type2_Devastator/ROS2/devas_ws/install/motor_controller'
