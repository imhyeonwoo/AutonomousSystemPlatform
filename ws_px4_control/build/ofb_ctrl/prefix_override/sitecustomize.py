import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ihw/workspace/AutonomousVehiclePlatform/ws_px4_control/install/ofb_ctrl'
