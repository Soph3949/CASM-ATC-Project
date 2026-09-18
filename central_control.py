import time  
import cflib.crtp  
from cflib.crazyflie.log import LogConfig  
from cflib.crazyflie.syncLogger import SyncLogger  
from cflib.swarm import Swarm  
from cflib.positioning.motion_commander import MotionCommander  
  
drone_1_uri = "radio://0/100/2M/E7E7E7E7E7"  
drone_2_uri = "radio://0/80/2M/E7E7E7E702"  
  
safety_radius = 0.40  
low_altitude_layer = 0.60  
high_altitude_layer = 0.90  
  
uris = [drone_1_uri]  
  
  
def reset_estimator(scf):  
    """Reset the Kalman estimator and wait for the position to converge."""  
    cf = scf.cf  
    cf.param.set_value("kalman.resetEstimation", "1")  
    time.sleep(0.1)  
    cf.param.set_value("kalman.resetEstimation", "0")  
  
    print(f"[{cf.link_uri}] Waiting for position estimate to converge...")  
  
    log_config = LogConfig(name="Kalman Variance", period_in_ms=100)  
    log_config.add_variable("kalman.varPX", "float")  
    log_config.add_variable("kalman.varPY", "float")  
    log_config.add_variable("kalman.varPZ", "float")  
  
    var_y_history = [1000.0] * 10  
    var_x_history = [1000.0] * 10  
    var_z_history = [1000.0] * 10  
    threshold = 0.001  
  
    with SyncLogger(scf, log_config) as logger:  
        for log_entry in logger:  
            data = log_entry[1]  
            var_x_history.append(data["kalman.varPX"])  
            var_x_history.pop(0)  
            var_y_history.append(data["kalman.varPY"])  
            var_y_history.pop(0)  
            var_z_history.append(data["kalman.varPZ"])  
            var_z_history.pop(0)  
  
            min_x, max_x = min(var_x_history), max(var_x_history)  
            min_y, max_y = min(var_y_history), max(var_y_history)  
            min_z, max_z = min(var_z_history), max(var_z_history)  
  
            if (max_x - min_x) < threshold and \  
               (max_y - min_y) < threshold and \  
               (max_z - min_z) < threshold:  
                print(f"[{cf.link_uri}] Position estimate converged.")  
                break  
  
  
def run_central_control_tasks(scf):  
    cf = scf.cf  
    print(f"[{cf.link_uri}] Connected successfully! Preparing for flight...")  
  
    # Make sure lighthouse positioning has a valid, converged estimate first.  
    reset_estimator(scf)  
  
    with MotionCommander(cf) as mc:  
        if cf.link_uri == drone_1_uri:  
            target_altitude = low_altitude_layer  
        else:  
            target_altitude = high_altitude_layer  
  
        print(f"[{cf.link_uri}] Taking off to altitude: {target_altitude}m")  
        mc.take_off(target_altitude)  
  
        time.sleep(3)  
  
        print(f"[{cf.link_uri}] Landing...")  
        mc.land()  
  
  
if __name__ == "__main__":  
    cflib.crtp.init_drivers()  
    print("Connecting to the swarm...")  
    with Swarm(uris) as swarm:  
        swarm.reset_estimators()  
        swarm.parallel_safe(run_central_control_tasks)  
        print("Swarm execution finished. Disconnecting all drones.")
