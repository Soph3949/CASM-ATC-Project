import math, threading, time  
import cflib.crtp  
from cflib.crazyflie.log import LogConfig  
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie  
from cflib.positioning.motion_commander import MotionCommander  
  
URI = 'radio://0/80/2M/E7E7E7E702'  
  
pos = [0.0, 0.0, 0.6]  
virt = {"pos": [0.0, 0.0, 0.0], "vel": [-0.4, 0.0, 0.0], "done": False}  
stop = threading.Event()  
  
def cb(timestamp, data, logconf):  
    pos[0], pos[1], pos[2] = (data['stateEstimate.x'],  
                              data['stateEstimate.y'], data['stateEstimate.z'])  
  
def monitor(mc):  
    last = time.time()  
    while not stop.is_set():  
        now = time.time()  
        dt = now - last  
        last = now  
        # move virtual drone toward us (head-on)  
        for i in range(3):  
            virt["pos"][i] += virt["vel"][i] * dt  
  
        dist = math.dist(pos, virt["pos"])  
        if dist < 0.3 and not virt["done"]:  
            virt["done"] = True  
            print(f"[ALERT] breach at {dist:.2f}m — evading!")  
            virt["vel"][2] = -0.3          # virtual drone dips down  
            mc.up(0.3, velocity=0.3)       # physical drone climbs  
            time.sleep(1.5)  
            mc.down(0.3, velocity=0.3)  
        time.sleep(0.1)  
  
if __name__ == '__main__':  
    cflib.crtp.init_drivers()  
    with SyncCrazyflie(URI) as scf:  
        lc = LogConfig(name='Position', period_in_ms=50)  
        for v in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z'):  
            lc.add_variable(v, 'float')  
        scf.cf.log.add_config(lc)  
        lc.data_received_cb.add_callback(cb)  
        lc.start()  
        time.sleep(1)   # let a fix arrive  
  
        scf.cf.platform.send_arming_request(True)  
        time.sleep(1)  
  
        with MotionCommander(scf, default_height=0.6) as mc:  
            time.sleep(2)  
            # intruder spawns 2m ahead on our line, flying at us  
            virt["pos"] = [pos[0] + 2.0, pos[1], pos[2]]  
            t = threading.Thread(target=monitor, args=(mc,)); t.start()  
            mc.forward(1.5, velocity=0.5)  
            stop.set(); t.join()  
  
        lc.stop()
