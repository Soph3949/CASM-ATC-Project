import logging
import math
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# Your Drone Radio URI
URI = 'radio://0/80/2M/E7E7E7E702'

logging.basicConfig(level=logging.ERROR)

# 1. Defined Virtual Drone Coordinates (X, Y, Z in meters)
VIRTUAL_COORDS = [
    (0.4, 0.6, 0.6),
    (1.4, 1.6, 0.6),
    (0.8, 0.2, 0.6),
]

SAFETY_RADIUS = 0.3  # 300 mm safety threshold
current_pos = [0.0, 0.0, 0.6]  # Live [x, y, z] position holder


def position_callback(timestamp, data, logconf):
    """Callback to update live position from Lighthouse system."""
    global current_pos
    current_pos[0] = data['stateEstimate.x']
    current_pos[1] = data['stateEstimate.y']
    current_pos[2] = data['stateEstimate.z']


def check_and_avoid(mc):
    """Calculates distance to virtual targets and executes pop-up if too close."""
    for i, coord in enumerate(VIRTUAL_COORDS):
        # Calculate 3D distance
        dist = math.sqrt(
            (current_pos[0] - coord[0]) ** 2 +
            (current_pos[1] - coord[1]) ** 2 +
            (current_pos[2] - coord[2]) ** 2
        )

        # IF distance < 300mm, THEN execute vertical pop-up maneuver
        if dist < SAFETY_RADIUS:
            print(f"[ATC ALERT] Proximity breach with Virtual Hazard {i+1}! Distance: {dist:.2f}m")
            print("Executing automated vertical evasion (Climbing +0.3m)...")
            
            mc.up(0.3, velocity=0.3)
            time.sleep(2)

            print("Hazard cleared. Returning to cruise altitude...")
            mc.down(0.3, velocity=0.3)
            time.sleep(1)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI) as scf:
        # Set up Lighthouse position tracking
        log_config = LogConfig(name='Position', period_in_ms=50)
        log_config.add_variable('stateEstimate.x', 'float')
        log_config.add_variable('stateEstimate.y', 'float')
        log_config.add_variable('stateEstimate.z', 'float')

        scf.cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(position_callback)
        log_config.start()

        # Arm drone
        scf.cf.platform.send_arming_request(True)
        time.sleep(1)

        # Execute flight path with MotionCommander
        with MotionCommander(scf, default_height=0.6) as mc:
            print("Taking off...")
            time.sleep(2)

            # Check safety at starting position
            check_and_avoid(mc)

            print("Flying forward...")
            mc.forward(1.5, velocity=0.5)
            time.sleep(1)

            # Check safety at end position
            check_and_avoid(mc)

            print("Landing...")
            time.sleep(1)

        log_config.stop()
