
import logging
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URI = 'radio://0/80/2M/E7E7E7E702'

# Conservative diagnostic settings
FLIGHT_HEIGHT = 0.3
FORWARD_SPEED = 0.3
OBSTACLE_DISTANCE = 0.5

CLIMB_HEIGHT = 0.4
CROSS_DISTANCE = 0.5
CEILING_CLEARANCE = 0.3

FLIGHT_TIME = 10

logging.basicConfig(level=logging.ERROR)


def obstacle_avoidance(mc, multiranger):
    start_time = time.time()

    print("Flying forward. Monitoring for obstacles...")

    try:
        while time.time() - start_time < FLIGHT_TIME:
            front = multiranger.front

            if front is None:
                # Unknown front distance: stop rather than assume clear.
                mc.stop()
                print("Front sensor unavailable. Aborting.")
                return

            if front < OBSTACLE_DISTANCE:
                print(f"Obstacle detected: {front:.2f} m")

                # Stop forward motion.
                mc.stop()
                time.sleep(0.5)

                # Check that the overhead distance is valid.
                above = multiranger.up

                if (
                    above is None
                    or above < CLIMB_HEIGHT + CEILING_CLEARANCE
                ):
                    print("Insufficient or unknown overhead clearance.")
                    return

                print("Climbing slowly...")
                mc.up(CLIMB_HEIGHT, velocity=0.1)
                mc.stop()
                time.sleep(0.5)

                # Recheck clearance after climbing.
                above = multiranger.up

                if above is None or above < CEILING_CLEARANCE:
                    print("Clearance check failed. Aborting.")
                    return

                print("Crossing a short distance...")
                mc.forward(1.5, velocity=0.1)
                mc.land()

                print("Crossing test complete.")
                print("Ending maneuver; no further movement commands.")
                return

            # Maintain a slow forward command while searching.
            mc.start_forward(velocity=FORWARD_SPEED)
            time.sleep(0.1)

        print("Flight time limit reached.")

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

                    obstacle_avoidance(mc, multiranger)

                    print("Landing through MotionCommander context.")

    except KeyboardInterrupt:
        print("Flight interrupted by user.")

