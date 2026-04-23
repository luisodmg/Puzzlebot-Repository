import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/dev_ws/src/manchester/te3002b_intelligent_robotics/week2/install/week2_puzzlebot_control'
