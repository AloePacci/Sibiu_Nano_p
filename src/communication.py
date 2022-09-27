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
    """this function manages the connection to the submarine
    @params:
        -ip: ip of the submarine
        -port: port to connect to
        -timeout: timeout for connection success
        -source system: ID for the program in MAVLink
        -source component: Component ID for the program in MAVLink
    @returns:
        it creates the shared variable self.vehicle
    @notes:
        if connection fails, it will log and notice reason of fail, killing the program
    """

    def __init__(self, logger=None, ip="192.168.2.1", port="8003", timeout=6.0, source_system=36, source_component=93):     
        if logger is None:
            self.log=Logger()
        else:
            self.log=logger   
        try:
            self.vehicle = connect(ip+":"+port,wait_ready=False, baud=115200, timeout=timeout, source_system=source_system, source_component=source_component)
        except ConnectionRefusedError:
            self.log.error("Connection to submarine was refused")
        except OSError:
            self.log.error("Sibiu was not found in the same network")
        except TimeoutError:
            self.log.error("Submarine connection timeout, port is busy")
        except:
            self.log.error(f"Connectiom could not be made, unknown error:\n")

        self.log.print(f"Connection SUCCESS")  



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
        self.log.log(f"mode changed to {mode}")
        self.vehicle.mode = VehicleMode(mode)

    def get_vehicle(self):
        return self.vehicle

    def safe_close(self):
        self.vehicle.mode = VehicleMode("MANUAL")
        self.vehicle.arm()

    def emergency_close(self):
        self.vehicle.disarm()
        self.vehicle.mode = VehicleMode("MANUAL")

if __name__ == '__main__':
    endopoint=Message_sender()