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
  
SAFETY_RADIUS = 0.3          # 300 mm safety threshold  
current_pos = [0.0, 0.0, 0.6]  # Live [x, y, z] position holder  
got_fix = False              # True once Lighthouse data arrives  
  
# Single virtual intruder — head-on conflict (180 deg approach angle)  
virtual_drone = {"pos": [0.0, 0.0, 0.0], "vel": [0.0, 0.0, 0.0], "evading": False}  
stop_monitor = threading.Event()  
  
  
def position_callback(timestamp, data, logconf):  
    """Callback to update live position from Lighthouse system."""  
    global current_pos, got_fix  
    current_pos[0] = data['stateEstimate.x']  
    current_pos[1] = data['stateEstimate.y']  
    current_pos[2] = data['stateEstimate.z']  
    got_fix = True  
  
  
def build_virtual_drone():  
    """Place the intruder ahead on the SAME flight line, flying toward us.  
  
    Real drone:  (0,0,0.6) -> (1.5,0,0.6) at 0.5 m/s along +X  
    Virtual:     starts ~2.0m ahead on y=0, flies -X at 0.4 m/s  
    -> head-on closure speed = 0.5 + 0.4 = 0.9 m/s (max possible)  
    """  
    sx, sy, sz = current_pos  
    print(f"Measured start position: ({sx:.2f}, {sy:.2f}, {sz:.2f})")  
  
    virtual_drone["pos"] = [sx + 2.0, sy, sz]   # dead ahead, same y, same z  
    virtual_drone["vel"] = [-0.4, 0.0, 0.0]     # flying straight at us (-X)  
    virtual_drone["evading"] = False  
  
    print(f"Virtual drone (HEAD-ON): start={virtual_drone['pos']}, "  
          f"vel={virtual_drone['vel']}, closure speed ~0.9 m/s")  
  
  
def distance_to(pos):  
    return math.sqrt(  
        (current_pos[0] - pos[0]) ** 2 +  
        (current_pos[1] - pos[1]) ** 2 +  
        (current_pos[2] - pos[2]) ** 2  
    )  
  
  
def monitor_thread(mc):  
    """Advances the virtual drone and checks for breaches at ~10Hz."""  
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
  
        # Live distance printout — watch it shrink fast  
        if now - last_print > 0.5:  
            print(f"  head-on closure: {dist:.2f}m")  
            last_print = now  
  
        if dist < SAFETY_RADIUS and not virtual_drone["evading"]:  
            virtual_drone["evading"] = True  
            print(f"[ATC ALERT] Head-on breach! Distance: {dist:.2f}m")  
            print("Executing automated vertical evasion (Climbing +0.3m)...")  
  
            mc.up(0.3, velocity=0.3)  
            time.sleep(1.5)  # let the intruder pass underneath  
  
            print("Hazard cleared. Returning to cruise altitude...")  
            mc.down(0.3, velocity=0.3)  
  
        # Reset flag once clear so a re-approach can trigger again  
        if dist > 1.5 * SAFETY_RADIUS:  
            virtual_drone["evading"] = False  
  
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
  
            # Intruder spawns ahead on our exact flight line, coming at us  
            build_virtual_drone()  
  
            # Start continuous collision monitoring  
            stop_monitor.clear()  
            monitor = threading.Thread(target=monitor_thread, args=(mc,))  
            monitor.start()  
  
            print("Flying forward... (head-on traffic inbound)")  
            mc.forward(1.5, velocity=0.5)  
            time.sleep(1)  
  
            # Stop monitoring before landing  
            stop_monitor.set()  
            monitor.join()  
  
            print("Landing...")  
            time.sleep(1)  
  
        log_config.stop()
