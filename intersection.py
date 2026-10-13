import logging  
import math  
import threading  
import time  
  
import cflib.crtp  
from cflib.crazyflie.log import LogConfig  
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie  
from cflib.positioning.motion_commander import MotionCommander  
  
# Your Drone Radio URI  
URI = 'radio://0/80/2M/E7E7E7E702'  
  
logging.basicConfig(level=logging.ERROR)  
  
SAFETY_RADIUS = 0.3            # 300 mm safety threshold  
CONFLICT_RADIUS = 0.6          # wider radius — detect early, yield before breach  
current_pos = [0.0, 0.0, 0.6]  # Live [x, y, z] position holder  
got_fix = False                # True once Lighthouse data arrives  
  
# Single virtual drone — 90-degree converging conflict (aerial crossroad)  
virtual_drone = {"pos": [0.0, 0.0, 0.0], "vel": [0.0, 0.0, 0.0], "yielded": False}  
stop_monitor = threading.Event()  
  
  
def position_callback(timestamp, data, logconf):  
    """Callback to update live position from Lighthouse system."""  
    global current_pos, got_fix  
    current_pos[0] = data['stateEstimate.x']  
    current_pos[1] = data['stateEstimate.y']  
    current_pos[2] = data['stateEstimate.z']  
    got_fix = True  
  
  
def build_virtual_drone():  
    """Place the intruder to the side, crossing our path at 90 degrees.  
  
    Real drone:  (0,0,0.6) -> (1.5,0,0.6) at 0.5 m/s along +X  
    Virtual:     starts ~0.9m to the right at x=0.75 (mid-intersection),  
                 flies -Y at 0.3 m/s -> perpendicular crossing, timed to  
                 arrive at the intersection as we do.  
    """  
    sx, sy, sz = current_pos  
    print(f"Measured start position: ({sx:.2f}, {sy:.2f}, {sz:.2f})")  
  
    # Intersection point: 0.75m ahead on our flight line.  
    # We reach it at t=1.5s into the forward leg (0.75 / 0.5).  
    # Intruder reaches y=0 at 0.9 / 0.3 = 3.0s -> starts before forward leg.  
    virtual_drone["pos"] = [sx + 0.75, sy + 0.9, sz]  
    virtual_drone["vel"] = [0.0, -0.3, 0.0]     # crossing at 90 degrees  
    virtual_drone["yielded"] = False  
  
    print(f"Virtual drone (CROSSING): start={virtual_drone['pos']}, "  
          f"vel={virtual_drone['vel']}, intersection at x={sx + 0.75:.2f}")  
  
  
def distance_to(pos):  
    return math.sqrt(  
        (current_pos[0] - pos[0]) ** 2 +  
        (current_pos[1] - pos[1]) ** 2 +  
        (current_pos[2] - pos[2]) ** 2  
    )  
  
  
def monitor_thread(mc):  
    """Advances the virtual drone and resolves the crossing conflict at ~10Hz."""  
    last_time = time.time()  
    last_print = 0.0  
  
    while not stop_monitor.is_set():  
        now = time.time()  
        dt = now - last_time  
        last_time = now  
  
        # Advance virtual drone (simple kinematics)  
        virtual_drone["pos"][0] += virtual_drone["vel"][0] * dt  
        virtual_drone["pos"][1] += virtual_drone["vel"][1] * dt  
        virtual_drone["pos"][2] += virtual_drone["vel"][2] * dt  
  
        dist = distance_to(virtual_drone["pos"])  
  
        # Live distance printout — watch it shrink  
        if now - last_print > 0.5:  
            print(f"  cross-traffic distance: {dist:.2f}m")  
            last_print = now  
  
        # Intersection resolution: YIELD — stop and hover, let traffic pass.  
        # Uses the wider CONFLICT_RADIUS so we brake BEFORE the 0.3m breach.  
        if dist < CONFLICT_RADIUS and not virtual_drone["yielded"]:  
            virtual_drone["yielded"] = True  
            print(f"[ATC] Crossing conflict! Distance: {dist:.2f}m")  
            print("Executing tactical deconfliction: holding position, "  
                  "yielding to cross-traffic...")  
  
            mc.stop()              # interrupt forward leg, hover in place  
            time.sleep(1.5)        # let the intruder clear the intersection  
  
            print("Intersection clear. Resuming flight...")  
            mc.forward(0.75, velocity=0.5)  # cover the remaining leg  
  
        time.sleep(0.1)  
  
  
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
  
        # Wait for a real Lighthouse position fix (max 5s)  
        print("Waiting for Lighthouse position fix...")  
        t0 = time.time()  
        while not got_fix:  
            if time.time() - t0 > 5:  
                print("ERROR: No Lighthouse position fix — "  
                      "check deck setup in cfclient. Aborting.")  
                log_config.stop()  
                raise SystemExit(1)  
            time.sleep(0.1)  
        print(f"Position fix acquired: {current_pos}")  
  
        # Arm drone  
        scf.cf.platform.send_arming_request(True)  
        time.sleep(1)  
  
        # Execute flight path with MotionCommander  
        with MotionCommander(scf, default_height=0.6) as mc:  
            print("Taking off...")  
            time.sleep(2)  
  
            # Intruder spawns to the side, converging at 90 degrees  
            build_virtual_drone()  
  
            # Start continuous conflict monitoring  
            stop_monitor.clear()  
            monitor = threading.Thread(target=monitor_thread, args=(mc,))  
            monitor.start()  
  
            print("Flying forward... (cross-traffic converging)")  
            mc.forward(1.5, velocity=0.5)  
            time.sleep(1)  
  
            # Stop monitoring before landing  
            stop_monitor.set()  
            monitor.join()  
  
            print("Landing...")  
            time.sleep(1)  
  
        log_config.stop()
