import time  

import cflib.crtp  
from cflib.crazyflie import Crazyflie  
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie  
from cflib.positioning.motion_commander import MotionCommander  

cflib.crtp.init_drivers()

URI = 'radio://0/80/2M/E7E7E7E702"

with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
    print("Connected")
    with MotionCommander(scf, default_height=0.3) as mc:  
        time.sleep(1) 
        mc.up(0.3)
        time.sleep(1)
        mc.forward(0.2)
        time.sleep(1)
        mc.down(0.3)
        time.sleep(1)

