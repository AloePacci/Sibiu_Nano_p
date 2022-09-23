from dronekit import connect, VehicleMode, LocationGlobal
import pymavlink
import numpy as np
import math
import traceback
from math import atan2
from pymavlink.dialects.v20 import ardupilotmega as mavlink2 
from pymavlink.quaternion import QuaternionBase
import os
os.environ["MAVLINK20"] = "1"
import time
from logger import Logger


class Message_sender:
    def __init__(self, vehiculo=None, logger=None):        
        if vehiculo is not None:
            self.vehicle=vehiculo
        else: 
            try:
                self.vehicle = connect("192.168.2.1:8002",wait_ready=False, baud=115200, timeout=6.0, source_system=36, source_component=93)
            except ConnectionRefusedError:
                print(f"Connection refused")
                print("Log module is dead")
                return
            except OSError:
                error = traceback.format_exc()
                print(f"not found in the same network \n\n{error}")
                print("Log module is dead")
                return
            except TimeoutError:
                print(f"port was busy, timeout error")
                print("Log module is dead")
                return
            except:
                error = traceback.format_exc()
                print(f"Connectiom could not be made, unknown error:\n {error}")
                print("Log module is dead")

        if logger is None:
            self.log=Logger(vehiculo=self.vehicle)
        else:
            self.log=logger

    def goto_deep(self, deepness):
        self.log.log(f"asked to go to deep {deepness}")
        msg=mavlink2.MAVLink_set_position_target_global_int_message(
            0, 0, 0,  # time (not used), target system, target component
            0, #MAV_FRAME_GLOBAL
            0b110111111000,  # yaw_rate,yaw,unused,Az,Ay,Ax,Vz,Vy,Vx,Z,Y,X
            0,  # lat e7    
            0,  # lon e7
            int(deepness),
            0,0,0, #vx,vy,vz
            0,0,0, #ax,ay,az
            0,  # yaw
            0)  # yaw rate
        self.vehicle.send_mavlink(msg)

    def orientate(self, desired_yaw):
        self.log.log("asked to rotate {desired_yaw}")
        msg = mavlink2.MAVLink_set_attitude_target_message(
            0, 0, 0,  # time (not used), target system, target component
            0b00000111,  # attitude, throttle, uns,uns,uns,yawrate,pitchrate,rollrate
            QuaternionBase([math.radians(angle) for angle in (0,0,desired_yaw)]), #quaternion
            0,0,0,0)  # roll rate, pitch rate, yaw rate, thrust
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


    def play_tune(self, tune):
        self.log.log("asked to play tune {tune}")
        msg=mavlink2.MAVLink_play_tune_message(
        0, 0,  #target system, target component
        bytes('', 'utf-8'), #format
        bytes('', 'utf-8') #tune
        )
        self.vehicle.send_mavlink(msg)

    def arm(self):
        self.log.log("vehicle arm")
        self.vehicle.arm()

    def disarm(self):
        self.log.log("vehicle disarm")
        self.vehicle.disarm()    

    def setmode(self, mode):
        self.log.log(f"change mode to {mode}")
        self.vehicle.mode = VehicleMode(mode)
"""
aux=datetime.today()   
time_aux=aux.strftime("%m.%d.%Y..%H.%M")"""


if __name__ == '__main__':
    endopoint=Message_sender()