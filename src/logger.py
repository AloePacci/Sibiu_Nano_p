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
            self.vehicle.add_message_listener('ATTITUDE', self.attitude_read)
            self.vehicle.add_message_listener('SYS_STATUS', self.sys_read)
            self.vehicle.add_message_listener('POWER_STATUS', self.power_read)
            self.vehicle.add_message_listener('NAV_CONTROLLER_OUTPUT', self.nav_read)
            self.vehicle.add_message_listener('VFR_HUD', self.vfr_read)
            self.vehicle.add_message_listener('SERVO_OUTPUT_RAW', self.servo_read)
            self.vehicle.add_message_listener('RC_CHANNELS_RAW ', self.rc_read)
            self.vehicle.add_message_listener('SCALED_IMU2', self.imu_read)
            self.vehicle.add_message_listener('GLOBAL_POSITION_INT', self.pos_read)
            self.vehicle.add_message_listener('EKF_STATUS_REPORT', self.ekf_read)
            self.vehicle.add_message_listener('BATTERY_STATUS', self.battery_read)
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
    
        self.data=pd.DataFrame(columns=["pressure","differential_pressure","pressure_temperature","roll","pitch","yaw","rollspeed","pitchspeed","yawspeed","armed","ekf","state","heading","mode","Vcc","nav_roll","nav_pitch","nav_bearing","wp_dist","alt_error","airspeed","groundspeed","throttle","alt","climb","servo1","servo2","servo3","servo4","servo5","servo6","servo7","servo8","rc1","rc2","rc3","rc4","rc5","rc6","rc7","rc8","xacc","yacc","zacc","xgyro","ygyro","zgyro","xmag","ymag","zmag","lat","lon","altgps"])
        while True:
            if len(self.data)>500:
                self.save_data()
            time.sleep(1)

    def save_data(self): #may be better with pandas
        if self.once:
            self.data.to_csv("out.csv",mode="w")
            self.once=False
        else:
            self.data.to_csv("out.csv",mode="a", header=False)
        #empty dataframe
        self.data=pd.DataFrame(index=[self.data.index[-1]],columns=["pressure","differential_pressure","pressure_temperature","roll","pitch","yaw","rollspeed","pitchspeed","yawspeed","armed","ekf","state","heading","mode","Vcc","nav_roll","nav_pitch","nav_bearing","wp_dist","alt_error","airspeed","groundspeed","throttle","alt","climb","servo1","servo2","servo3","servo4","servo5","servo6","servo7","servo8","rc1","rc2","rc3","rc4","rc5","rc6","rc7","rc8","xacc","yacc","zacc","xgyro","ygyro","zgyro","xmag","ymag","zmag","lat","lon","altgps"])

    def pressure_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.press_abs, msg.press_diff, msg.temperature],columns=[msg.time_boot_ms]  ,index=["pressure","differential_pressure","pressure_temperature"]).T], sort=True)
    def attitude_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed],columns=[msg.time_boot_ms]  ,index=["roll","pitch","yaw","rollspeed","pitchspeed","yawspeed"]).T], sort=True)
    def sys_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([self.vehicle.armed, self.vehicle.ekf_ok, self.vehicle.system_status.state, self.vehicle.mode.name],columns=[self.data.index[-1]]  ,index=["armed","ekf","state","mode"]).T], sort=True)
        #self.data=pd.concat([self.data, pd.DataFrame([],columns=[self.data.index[-1]]  ,index=[]).T], sort=True)
    def power_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.Vcc],columns=[self.data.index[-1]]  ,index=["Vcc"]).T], sort=True)
    def nav_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.nav_roll, msg.nav_pitch, msg.nav_bearing, msg.wp_dist, msg.alt_error],columns=[self.data.index[-1]]  ,index=["nav_roll","nav_pitch","nav_bearing","wp_dist","alt_error"]).T], sort=True)
    def vfr_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb],columns=[self.data.index[-1]]  ,index=["airspeed","groundspeed","heading","throttle","alt","climb"]).T], sort=True)
    def servo_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw, msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw],columns=[msg.time_usec//1000]  ,index=["servo1","servo2","servo3","servo4","servo5","servo6","servo7","servo8"]).T], sort=True)
    def rc_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.chan1_raw , msg.chan2_raw , msg.chan3_raw ,msg.chan4_raw , msg.chan5_raw , msg.chan6_raw ,msg.chan7_raw , msg.chan8_raw],columns=[msg.time_boot_ms]  ,index=["rc1","rc2","rc3","rc4","rc5","rc6","rc7","rc8"]).T], sort=True)
    def imu_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro, msg.xmag, msg.ymag, msg.zmag],columns=[msg.time_boot_ms]  ,index=["xacc","yacc","zacc","xgyro","ygyro","zgyro","xmag","ymag","zmag"]).T], sort=True)
    def pos_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.lat, msg.lon, msg.alt],columns=[msg.time_boot_ms]  ,index=["lat","lon","altgps"]).T], sort=True)
    def ekf_read(self,vehicle, name, msg):
        pass
        #self.data=pd.concat([self.data, pd.DataFrame([msg.press_abs, msg.press_diff, msg.temperature],columns=[msg.time_boot_ms]  ,index=["pressure","differential_pressure","pressure_temperature"]).T], sort=True)
    def battery_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.temperature, msg.voltages[0], msg.current_consumed, msg.battery_remaining],columns=[self.data.index[-1]]  ,index=["bat_temp","bat_voltage","current","battery %"]).T], sort=True)

"""
aux=datetime.today()   
time_aux=aux.strftime("%m.%d.%Y..%H.%M")"""

if __name__ == '__main__':
    endopoint=Logger()