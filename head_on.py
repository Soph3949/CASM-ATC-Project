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
CLIMB_HEIGHT = 1      # Climb by 1 m to avoid obstacles
CEILING_CLEARANCE = 0.6  # Require 60 cm above the drone
FLIGHT_TIME = 30         # Maximum flight time in seconds (climbing 1 m takes ~7 s)
HEIGHT_JUMP = 0.15       # Down-sensor change that means the object starts/ends
PASS_DISTANCE = 0.2      # Extra distance to fly after the object so the drone is fully past it
LANDING_SPEED = 0.1      # m/s, slower than the default for a smooth landing

logging.basicConfig(level=logging.ERROR)


def obstacle_avoidance(mc, multiranger):
    start_time = time.time()
    flying_forward = False
    climbed = False
    over_obstacle = False
    last_below = None

    print("Flying forward. Monitoring for obstacles...")

    try:
        while time.time() - start_time < FLIGHT_TIME:
            front = multiranger.front
            above = multiranger.up
            below = multiranger.down

            # A valid front reading indicates an obstacle is nearby.
            # Only look for it before climbing; afterwards we are above it.
            obstacle_detected = (
                not climbed
                and front is not None
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
                climbed = True
                last_below = multiranger.down

                # Resume forward movement after climbing.
                print("Continuing forward. Watching for the object below...")
                mc.start_forward(velocity=FORWARD_SPEED)
                flying_forward = True

            elif climbed and below is not None and last_below is not None:
                # The down sensor suddenly gets shorter when the object
                # is under the drone, and suddenly longer once it is behind.
                if not over_obstacle and below < last_below - HEIGHT_JUMP:
                    print(f"Over the object: {below:.2f} m below")
                    over_obstacle = True

                elif over_obstacle and below > last_below + HEIGHT_JUMP:
                    print("Object cleared.")

                    # Fly a little further so the back of the drone is past it.
                    mc.forward(PASS_DISTANCE, velocity=FORWARD_SPEED)
                    mc.stop()
                    flying_forward = False
                    time.sleep(0.5)

                    # Go back down to the original altitude.
                    print("Returning to original altitude...")
                    mc.down(CLIMB_HEIGHT, velocity=0.15)
                    time.sleep(1)
                    break

            elif not flying_forward:
                mc.start_forward(velocity=FORWARD_SPEED)
                flying_forward = True

            if below is not None:
                last_below = below

            time.sleep(0.05)

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
                        time.sleep(1)
                        mc.land(velocity=LANDING_SPEED)

    except KeyboardInterrupt:
        print("Flight interrupted by user.")
