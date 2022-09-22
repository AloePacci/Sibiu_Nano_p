from dronekit import connect, VehicleMode, LocationGlobal
import pymavlink
import numpy as np
import traceback
from math import atan2
from pymavlink.dialects.v20 import ardupilotmega as mavlink2 #for obstacle distance information
from pymavlink.quaternion import QuaternionBase
import os
os.environ["MAVLINK20"] = "1"
import time


class Message_sender:
    def __init__(self):
        self.once=True
        # Send a ping to start connection and wait for any reply.

        try:
            self.vehicle = connect("192.168.2.1:8001",wait_ready=False, baud=115200, timeout=6.0, source_system=36, source_component=93)
            #self.vehicle.add_message_listener('SYS_STATUS', self.vehicle_status_callback)
            #self.cmds = self.vehicle.commands
            #self.cmds.download()
            #self.cmds.wait_ready()
            #self.home = self.vehicle.home_location
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
            return

        print(f"Connection SUCCESS")
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        #self.vehicle.arm()
        print(f"armed")
        #self.vehicle.play_tune("AAAA")
        for i in range(2):
            time.sleep(1)
        self.vehicle.disarm()
        self.vehicle.mode = VehicleMode("MANUAL")
        print(f"disarmed")



    def vehicle_status_callback(self, vehicle, name, msg):
        if self.once==True:
            print(f"{name} : {msg}")
            self.once=False

    def goto_deep(self, deepness):
        msg = self.vehicle.message_factory.SET_POSITION_TARGET_GLOBAL_INT(
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

        msg = self.vehicle.message_factory.SET_ATTITUDE_TARGET(
            0, 0, 0,  # time (not used), target system, target component
            0b00000111,  # attitude, throttle, uns,uns,uns,yawrate,pitchrate,rollrate
            QuaternionBase([math.radians(angle) for angle in (0,0,desired_yaw)]), #quaternion
            0,0,0,0)  # roll rate, pitch rate, yaw rate, thrust
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


    



"""
aux=datetime.today()   
time_aux=aux.strftime("%m.%d.%Y..%H.%M")"""


if __name__ == '__main__':
    endopoint=Message_sender()