import time
import logging
import math
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.crazyflie.mem import LighthouseMemHelper
from cflib.utils import uri_helper

def activate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 255)

def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 0)

def light_check(scf):
    print("lights on")
    activate_led_bit_mask(scf)
    time.sleep(6)
    deactivate_led_bit_mask(scf)
    time.sleep(6)
    print("lights off")

def take_off(scf):
    print("In take_off")
    commander = scf.cf.high_level_commander
    commander.takeoff(0.5, 10.0)
    time.sleep(10)
    print("end of take_off")

def land(scf):
    commander = scf.cf.high_level_commander
    print("in land")
    commander.land(0.0, 5.0)
    time.sleep(7)

    print("end of land")
def hover_sequence(scf):
    scf.cf.platform.send_arming_request(True)
    print("Calling take off")
    take_off(scf)
    print("Take off complete")
    land(scf)

uris = {
    'radio://0/100/2M/E7E7E7E701',
    'radio://0/100/2M/E7E7E7E702',
}

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        print('Connected to Crazyflies')
        swarm.reset_estimators()
        swarm.parallel_safe(hover_sequence)
