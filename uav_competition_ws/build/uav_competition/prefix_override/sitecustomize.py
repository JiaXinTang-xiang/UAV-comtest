import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jiaxintang/桌面/bao/uav_competition_ws/install/uav_competition'
