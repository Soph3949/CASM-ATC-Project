import time  

import cflib.crtp  
from cflib.crazyflie import Crazyflie  
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie  
from cflib.positioning.motion_commander import MotionCommander  

URI = "radio://0/100/2M/E7E7E7E7E7"

cflib.crtp.init_drivers()

print(cflib.crtp.scan_interfaces())

with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:  
    with MotionCommander(scf, default_height=0.5) as mc:  
        time.sleep(1)
        
        mc.up(0.3)
        time.sleep(1)

        mc.forward(0.2)
        time.sleep(1)

        mc.down(0.3)
        time.sleep(1)
