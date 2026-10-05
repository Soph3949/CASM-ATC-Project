import threading  
import time  
  
import cflib.crtp  
from cflib.crazyflie import Crazyflie  
from cflib.crazyflie.log import LogConfig  
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie  
from cflib.crazyflie.syncLogger import SyncLogger  
from cflib.positioning.motion_commander import MotionCommander  
from cflib.crazyflie.swarm import Swarm  
  
DRONE_1_URI = "radio://0/100/2M/E7E7E7E7E7"  
DRONE_2_URI = "radio://0/80/2M/E7E7E7E702"  
  
CRUISE_ALTITUDE = 0.6
SAFETY_RADIUS   = 0.40  
RESUME_RADIUS   = 0.60
DODGE_HEIGHT    = 0.30 
  
BOUNDS = {"x": (0.2, 1.8), "y": (0.2, 1.8), "z": (0.3, 1.2)}  
  
LANDMARKS = {  
    "kanata":     (0.4, 1.2),   # west side  
    "nepean":     (0.4, 0.4),   # southwest corner  
    "parliament": (1.6, 1.0),   # east side (downtown)  
    "charge_1": (0.5,0.8)       # charging station 1
    "charge_2": (1.5,1.8)       # charging station 2
}
  
MISSIONS = {  
    DRONE_1_URI: ["kanata", "parliament"],  
    DRONE_2_URI: ["nepean", "kanata"],  
}  
  
VELOCITY = 0.3 
STEP = 0.1 
   
positions = {}  
pos_lock = threading.Lock()  
  
  
def clamp(v, lo, hi):  
    return max(lo, min(hi, v))  
  
  
def make_pos_logger(scf):  
    lg = LogConfig(name="Position", period_in_ms=100)  
    lg.add_variable("stateEstimate.x", "float")  
    lg.add_variable("stateEstimate.y", "float")  
    lg.add_variable("stateEstimate.z", "float")  
  
    def cb(ts, data, logconf):  
        with pos_lock:  
            positions[scf.cf.link_uri] = (  
                data["stateEstimate.x"],  
                data["stateEstimate.y"],  
                data["stateEstimate.z"],  
            )  
  
    lg.data_received_cb.add_callback(cb)  
    scf.cf.log.add_config(lg)  
    lg.start()  
    return lg  
  
  
def reset_estimator(scf):  
    """Reset the Kalman estimator and wait for position to converge."""  
    cf = scf.cf  
    cf.param.set_value("kalman.resetEstimation", "1")  
    time.sleep(0.1)  
    cf.param.set_value("kalman.resetEstimation", "0")  
  
    lg = LogConfig(name="Kalman Variance", period_in_ms=100)  
    for var in ("kalman.varPX", "kalman.varPY", "kalman.varPZ"):  
        lg.add_variable(var, "float")  
  
    hist = [[1000.0] * 10 for _ in range(3)]  
    with SyncLogger(scf, lg) as logger:  
        for entry in logger:  
            d = entry[1]  
            for i, var in enumerate(("kalman.varPX", "kalman.varPY", "kalman.varPZ")):  
                hist[i].append(d[var])  
                hist[i].pop(0)  
            if all(max(h) - min(h) < 0.001 for h in hist):  
                print(f"[{cf.link_uri}] Position estimate converged.")  
                return  
  
  
def horizontal_distance(uri_a, uri_b):  
    with pos_lock:  
        a, b = positions.get(uri_a), positions.get(uri_b)  
    if not a or not b:  
        return float("inf") 
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5  
  
  
def other_uri(my_uri):  
    return DRONE_2_URI if my_uri == DRONE_1_URI else DRONE_1_URI  
  
  
def fly_mission(scf):  
    cf = scf.cf  
    my_uri = cf.link_uri  
    peer = other_uri(my_uri)  
    i_dodge_up = (my_uri == DRONE_1_URI)  
  
    reset_estimator(scf)  
    pos_logger = make_pos_logger(scf)  
  
    try:  
        with MotionCommander(cf, default_height=CRUISE_ALTITUDE) as mc:  
            print(f"[{my_uri}] Takeoff to {CRUISE_ALTITUDE} m")  
            time.sleep(1.0)  
  
            for landmark in MISSIONS[my_uri]:  
                tx, ty = LANDMARKS[landmark]  
                tx = clamp(tx, *BOUNDS["x"])   # boundary failsafe  
                ty = clamp(ty, *BOUNDS["y"])  
                print(f"[{my_uri}] -> {landmark} ({tx:.1f}, {ty:.1f})")  
  
                # Fly the leg in STEP-sized segments so we can react.  
                while True:  
                    with pos_lock:  
                        mx, my, _ = positions.get(my_uri, (tx, ty, 0))  
                    dx, dy = tx - mx, ty - my  
                    dist_left = (dx * dx + dy * dy) ** 0.5  
                    if dist_left < STEP:  
                        break  
  
                    # --- IF-THEN rule 1: collision avoidance ---  
                    if horizontal_distance(my_uri, peer) < SAFETY_RADIUS:  
                        dz = DODGE_HEIGHT if i_dodge_up else -DODGE_HEIGHT  
                        z = clamp(CRUISE_ALTITUDE + dz, *BOUNDS["z"])  
                        print(f"[{my_uri}] AVOIDING -> z={z:.2f}")  
                        mc.go_to(mx, my, z)      # hold X/Y, change Z  
                        while horizontal_distance(my_uri, peer) < RESUME_RADIUS:  
                            time.sleep(0.1)  
                        print(f"[{my_uri}] RESUMING cruise altitude")  
                        mc.go_to(mx, my, CRUISE_ALTITUDE)  
                        time.sleep(0.5)  
                        continue  
  
                    # --- IF-THEN rule 2: boundary clamp on every setpoint ---  
                    step = min(STEP, dist_left)  
                    nx = clamp(mx + dx / dist_left * step, *BOUNDS["x"])  
                    ny = clamp(my + dy / dist_left * step, *BOUNDS["y"])  
                    mc.go_to(nx, ny, CRUISE_ALTITUDE, velocity=VELOCITY)  
  
            print(f"[{my_uri}] Mission complete. Landing...")  
            mc.land()  
    finally:  
        # --- IF-THEN rule 3: guaranteed landing on any failure/link loss ---  
        pos_logger.stop()  
        try:  
            cf.commander.send_stop_setpoint()  
        except Exception:  
            pass  
        print(f"[{my_uri}] Safely shut down.")  

if __name__ == "__main__":  
    cflib.crtp.init_drivers()  
    uris = [DRONE_1_URI, DRONE_2_URI]  
    with Swarm(uris) as swarm:  
        swarm.parallel_safe(fly_mission)
