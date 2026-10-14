import logging
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URI = 'radio://0/80/2M/E7E7E7E702'

# Flight settings
FLIGHT_HEIGHT = 0.3
FORWARD_SPEED = 0.2      # m/s
OBSTACLE_DISTANCE = 0.3   # Stop if an object is closer than this
CLIMB_HEIGHT = 0.3      # Climb by 30 cm
CEILING_CLEARANCE = 0.6  # Require 60 cm above the drone
FLIGHT_TIME = 10         # Maximum forward-flight time in seconds

logging.basicConfig(level=logging.ERROR)


def obstacle_avoidance(mc, multiranger):
    start_time = time.time()
    flying_forward = False

    print("Flying forward. Monitoring for obstacles...")

    try:
        while time.time() - start_time < FLIGHT_TIME:
            front = multiranger.front
            above = multiranger.up

            # A valid front reading indicates an obstacle is nearby.
            obstacle_detected = (
                front is not None
                and front < OBSTACLE_DISTANCE
            )

            if obstacle_detected:
                print(f"Obstacle detected: {front:.2f} m")

                # Stop horizontal movement before attempting to climb.
                mc.stop()
                flying_forward = False
                time.sleep(0.3)

                # Check whether the upward sensor reports enough room.
                if above is not None and above < CEILING_CLEARANCE:
                    print("Not enough overhead clearance. Landing.")
                    break

                print("Climbing to avoid obstacle...")
                mc.up(CLIMB_HEIGHT, velocity=0.15)
                time.sleep(0.3)

                # Resume forward movement after climbing.
                print("Continuing forward...")
                mc.start_forward(velocity=FORWARD_SPEED)
                flying_forward = True

            elif not flying_forward:
                mc.start_forward(velocity=FORWARD_SPEED)
                flying_forward = True

            time.sleep(0.05)

    finally:
        # Always stop commanded forward motion before exiting.
        mc.stop()


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    try:
        with SyncCrazyflie(URI) as scf:
            scf.cf.platform.send_arming_request(True)
            time.sleep(1)

            # Requires a compatible Multiranger deck.
            with Multiranger(scf) as multiranger:
                with MotionCommander(
                    scf, default_height=FLIGHT_HEIGHT
                ) as mc:
                    print("Taking off...")
                    time.sleep(2)

                    try:
                        obstacle_avoidance(mc, multiranger)
                    finally:
                        print("Landing...")
                        mc.stop()
                        mc.land()

    except KeyboardInterrupt:
        print("Flight interrupted by user.")

