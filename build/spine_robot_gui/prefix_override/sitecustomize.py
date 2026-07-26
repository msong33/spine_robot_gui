import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/michelle/spine_robot_gui/install/spine_robot_gui'
