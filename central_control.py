import logging
import time
import cflib  # type: ignore
import cflib.crtp  # type: ignore
from cflib.swarm import Swarm  # type: ignore
from cflib.positioning.motion_commander import MotionCommander  # type: ignore

drone_1_uri = "radio://0/80/2M/E7E7E7E701"
drone_2_uri = "radio://0/80/2M/E7E7E7E702"
safety_radius = 0.40
low_altitude_layer = 0.60
high_altitude_layer = 0.90
live_airspace_registry = {}

uris = [drone_1_uri, drone_2_uri]

def run_central_control_tasks(scf):
    cf = scf.cf
    print(f"[{cf.link_uri}] Connected successfully! Starting test maneuver...")
    
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
    with Swarm(uris, factory=run_central_control_tasks) as swarm:
        print("Swarm execution finished. Disconnecting all drones.")