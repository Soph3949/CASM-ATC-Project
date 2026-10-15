
import logging
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URI = 'radio://0/80/2M/E7E7E7E702'

# Flight settings
FLIGHT_HEIGHT = 0.2
FORWARD_SPEED = 0.3
OBSTACLE_DISTANCE = 0.5

# Obstacle crossing settings
CLIMB_HEIGHT = 0.4    # Climb an additional 30 cm
CROSS_DISTANCE = 2.0     # Move forward 2 m at the higher altitude
CEILING_CLEARANCE = 0.3  # Extra clearance above the planned climb

FLIGHT_TIME = 10

logging.basicConfig(level=logging.ERROR)

def obstacle_avoidance(mc, multiranger):
    start_time = time.time()
    obstacle_handled = False

    print("Flying forward. Monitoring for obstacles...")

    try:
        while time.time() - start_time < FLIGHT_TIME:
            front = multiranger.front

            # Detect an obstacle in front of the drone.
            if (
                not obstacle_handled
                and front is not None
                and front < OBSTACLE_DISTANCE
            ):
                print(f"Obstacle detected: {front:.2f} m")

                # Stop before performing the maneuver.
                mc.stop()
                time.sleep(0.3)

                # Check overhead clearance.
                above = multiranger.up

                if (
                    above is None
                    or above < CLIMB_HEIGHT + CEILING_CLEARANCE
                ):
                    print(
                        "Overhead clearance insufficient or unknown. "
                        "Aborting the maneuver."
                    )
                    break

                print("Step 1: Climbing...")
                mc.up(CLIMB_HEIGHT, velocity=0.2)

                # Give the range sensor time to update.
                time.sleep(0.3)

                print("Step 2: Flying forward over the obstacle...")
                mc.forward(1, velocity=FORWARD_SPEED)

                mc.land()

                print("Obstacle crossing sequence completed.")

                obstacle_handled = True
            else:
                mc.start_forward(velocity=FORWARD_SPEED)

            time.sleep(0.1)

    finally:
        mc.stop()


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    try:
        with SyncCrazyflie(URI) as scf:
            scf.cf.platform.send_arming_request(True)
            time.sleep(1)

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

