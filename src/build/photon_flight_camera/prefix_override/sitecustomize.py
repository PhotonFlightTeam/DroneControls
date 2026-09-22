import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/punished_venom/DroneControls/src/install/photon_flight_camera'
