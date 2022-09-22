from pickle import FALSE
from dronekit import connect, VehicleMode, LocationGlobal
import pymavlink
import traceback
from math import atan2
from pymavlink.dialects.v20 import ardupilotmega as mavlink2 #for obstacle distance information
import time
import os
os.environ["MAVLINK20"] = "1"

import numpy as np
import pandas as pd


class Logger:
    def __init__(self):
        self.once=True
        try:
            self.vehicle = connect("192.168.2.1:8001", timeout=6.0, source_system=1, source_component=93)
            #listener for custom messages
            #self.vehicle.add_message_listener('*', self.vehicle_status_callback)
            self.vehicle.add_message_listener('SCALED_PRESSURE2', self.pressure_read)
            self.cmds = self.vehicle.commands
            self.cmds.download()
            self.cmds.wait_ready()
            self.home = self.vehicle.home_location
        except ConnectionRefusedError:
            print(f"Connection refused")
            print("Log module is dead")
        except OSError:
            print(f"not found in the same network")
            print("Log module is dead")
        except TimeoutError:
            print(f"port was busy, timeout error")
            print("Log module is dead")
        except:
            error = traceback.format_exc()
            print(f"Connectiom could not be made, unknown error:\n {error}")
            print("Log module is dead")
        self.init_lists()
        while True:
            self.arm.append(self.vehicle.armed)
            self.bat.append(self.vehicle.battery.voltage)
            self.chann.append(self.vehicle.channels)
            self.deep.append(self.vehicle.location.global_relative_frame.alt)
            self.sdif.append(0)
            self.state.append(self.vehicle.system_status.state)
            self.yaw.append(self.vehicle.attitude.yaw)
            self.heading.append(self.vehicle.heading)
            self.roll.append(self.vehicle.attitude.roll)
            self.pitch.append(self.vehicle.attitude.pitch)
            self.lat.append(self.vehicle.location.global_relative_frame.lat)
            self.lon.append(self.vehicle.location.global_relative_frame.lon)
            self.mode.append(self.vehicle.mode.name)
            self.ekf.append(self.vehicle.ekf_ok)
            self.speed.append(self.vehicle.groundspeed)
            self.airspeed.append(self.vehicle.airspeed)
            time.sleep(0.05)
            if len(self.arm)>100:
                self.save_data()

    def save_data(self): #may be better with pandas
        data=pd.DataFrame([self.arm, self.bat, self.chann, self.deep, self.pres, self.presd, self.sdif, self.state, self.yaw, self.heading, self.roll, self.pitch, self.lat, self.lon, self.mode, self.ekf, self.speed, self.airspeed],
        index=["armed", "battery", "channels", "deepness", "pressure", "differential_pressure", "sdif", "state", "yaw", "heading", "roll","pitch","lat","lon","mode","ekf","speed","airspeed"])
        data=data.transpose()
        if self.once:
            data.to_csv("out.csv",mode="w", index=False)
            self.once=False
        else:
            data.to_csv("out.csv",mode="a", index=False, header=False)
        self.init_lists()

    def init_lists(self):
            self.arm=[]
            self.bat=[]
            self.chann=[]
            self.deep=[]
            self.pres=[]
            self.presd=[]
            self.sdif=[]
            self.state=[]
            self.yaw=[]
            self.heading=[]
            self.roll=[]
            self.pitch=[]
            self.lat=[]
            self.lon=[]
            self.mode=[]
            self.ekf=[]
            self.speed=[]
            self.airspeed=[]
            self.pres_temp=[]


    def vehicle_status_callback(self, vehicle, name, msg):
        with open("messages.txt","a") as f:
            f.write(f"{name} : {msg}\n")    


    def pressure_read(self,vehicle, name, msg):
        self.pres.append(msg.press_abs)
        self.presd.append(msg.press_diff)
        self.pres_temp.append(msg.temperature)

"""
aux=datetime.today()   
time_aux=aux.strftime("%m.%d.%Y..%H.%M")"""

if __name__ == '__main__':
    endopoint=Logger()