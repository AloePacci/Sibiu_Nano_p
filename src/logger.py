import traceback
import time
import os
import numpy as np
import pandas as pd
import threading
from datetime import datetime
import inspect



class CustomError(Exception):
    pass

class Logger:
    def __init__(self, vehiculo=None):
        self.__once=True #variable to instanciate the file
        self.__stop=False #variable to stop logging to file
        aux=datetime.today()   
        self.filename=aux.strftime("data %m.%d.%Y..%H.%M")
        #instanciate columns so the order is always the same in the file.
        self.data=pd.DataFrame(columns=["pressure","differential_pressure","pressure_temperature","roll","pitch","yaw","rollspeed","pitchspeed","yawspeed","armed","ekf","state","heading","mode","Vcc","nav_roll","nav_pitch","nav_bearing","wp_dist","alt_error","airspeed","groundspeed","throttle","alt","climb","servo1","servo2","servo3","servo4","servo5","servo6","servo7","servo8","rc1","rc2","rc3","rc4","rc5","rc6","rc7","rc8","xacc","yacc","zacc","xgyro","ygyro","zgyro","xmag","ymag","zmag","lat","lon","altgps"])



        if vehiculo is None:
            self.vehicle=None
            #print("no vehicle found")
        else:
            self.append_vehicle(vehiculo)


    def append_vehicle(self, vehicle, outside=False):
        self.log("vehicle found, starting logging")
        self.vehicle=vehicle.vehicle
        self.gps_handler=vehicle.gps
        
        if outside:#listener for messages
            self.external_save_thread=threading.Thread(target=self.external_save)
            self.external_save_thread.start()


        else:
            self.vehicle.add_message_listener('SCALED_PRESSURE2', self.pressure_read)
            self.vehicle.add_message_listener('ATTITUDE', self.attitude_read)
            self.vehicle.add_message_listener('SYS_STATUS', self.sys_read)
            self.vehicle.add_message_listener('POWER_STATUS', self.power_read)
            self.vehicle.add_message_listener('NAV_CONTROLLER_OUTPUT', self.nav_read)
            self.vehicle.add_message_listener('VFR_HUD', self.vfr_read)
            self.vehicle.add_message_listener('SERVO_OUTPUT_RAW', self.servo_read)
            #self.vehicle.add_message_listener('RC_CHANNELS_RAW', self.rc_read)
            #self.vehicle.add_message_listener('SCALED_IMU2', self.imu_read)
            self.vehicle.add_message_listener('GLOBAL_POSITION_INT', self.pos_read)
            #self.vehicle.add_message_listener('EKF_STATUS_REPORT', self.ekf_read)
            self.vehicle.add_message_listener('BATTERY_STATUS', self.battery_read)
            #download vehicle params
            self.cmds = self.vehicle.commands
            self.cmds.download()
            self.cmds.wait_ready()
            self.home = self.vehicle.home_location
    
        #create thread to save data into file
        self.save_thread=threading.Thread(target=self.save_data)
        self.save_thread.start()

    def external_save(self):
        try:
            x=self.gps_handler.x
            y=self.gps_handler.y
        except:
            x=0
            y=0
        last=[self.vehicle.attitude.roll, self.vehicle.attitude.roll, self.vehicle.heading, self.vehicle.location.global_frame.alt, x]
        while not self.__stop:
            if last!=[self.vehicle.attitude.roll, self.vehicle.attitude.roll, self.vehicle.heading, self.vehicle.location.global_frame.alt,x]:
                last=[self.vehicle.attitude.roll, self.vehicle.attitude.roll, self.vehicle.heading, self.vehicle.location.global_frame.alt,x]
                a=datetime.now()
                timestamp=a.second+a.minute*60+a.microsecond/1000000
                self.data=pd.concat([self.data, pd.DataFrame([self.vehicle.attitude.roll, self.vehicle.attitude.roll, self.vehicle.heading, self.vehicle.location.global_frame.alt, x, y],columns=[timestamp]  ,index=["roll","pitch","heading","alt","lat","lon"]).T], sort=True)
                time.sleep(0.01)

        
    def save_data(self): 
        print("starting log")
        while not self.__stop:
            if len(self.data)>200: #save data once we have few data to save
                if self.__once:#first data?
                    self.data.to_csv("./data/"+self.filename+".csv",mode="w") #create file at start
                    self.__once=False
                else:
                    self.data.to_csv("./data/"+self.filename+".csv",mode="a", header=False) #update file for following data
                #empty dataframe
                self.data=pd.DataFrame(index=[self.data.index.max()],columns=["pressure","differential_pressure","pressure_temperature","roll","pitch","yaw","rollspeed","pitchspeed","yawspeed","armed","ekf","state","heading","mode","Vcc","nav_roll","nav_pitch","nav_bearing","wp_dist","alt_error","airspeed","groundspeed","throttle","alt","climb","servo1","servo2","servo3","servo4","servo5","servo6","servo7","servo8","rc1","rc2","rc3","rc4","rc5","rc6","rc7","rc8","xacc","yacc","zacc","xgyro","ygyro","zgyro","xmag","ymag","zmag","lat","lon","altgps"])
            time.sleep(1)
        self.data.to_csv("./data/"+self.filename+".csv",mode="a", header=False) #update file last data
        self.print("log save thread closed")


    def pressure_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.press_abs, msg.press_diff, msg.temperature],columns=[msg.time_boot_ms]  ,index=["pressure","differential_pressure","pressure_temperature"]).T], sort=True)
    def attitude_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed],columns=[msg.time_boot_ms]  ,index=["roll","pitch","yaw","rollspeed","pitchspeed","yawspeed"]).T], sort=True)
    def sys_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([int(self.vehicle.armed), int(self.vehicle.ekf_ok), self.vehicle.system_status.state, self.vehicle.mode.name],columns=[self.data.index.max()]  ,index=["armed","ekf","state","mode"]).T], sort=True)
        try:
            if self.last_arm!=self.vehicle.armed:
                self.log("vehicle was armed" if self.vehicle.armed else "vehicle was disarmed")
            if self.last_mode!=self.vehicle.mode.name:
                self.log(f"mode changed to {self.vehicle.mode.name}")
            self.last_arm=self.vehicle.armed
            self.last_mode=self.vehicle.mode.name
        except:
            self.last_arm=self.vehicle.armed
            self.last_mode=self.vehicle.mode.name
        #self.data=pd.concat([self.data, pd.DataFrame([],columns=[self.data.index[-1]]  ,index=[]).T], sort=True)
    def power_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.Vcc],columns=[self.data.index.max()]  ,index=["Vcc"]).T], sort=True)
    def nav_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.nav_roll, msg.nav_pitch, msg.nav_bearing, msg.wp_dist, msg.alt_error],columns=[self.data.index.max()]  ,index=["nav_roll","nav_pitch","nav_bearing","wp_dist","alt_error"]).T], sort=True)
    def vfr_read(self,vehicle, name, msg):
        self.data=pd.concat([self.data, pd.DataFrame([msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb],columns=[self.data.index.max()]  ,index=["airspeed","groundspeed","heading","throttle","alt","climb"]).T], sort=True)
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


    def stop_logging(self):
        self.__stop=True


    def print(self, message):
        print(message)
        stack = inspect.stack()
        if len(stack)>2:
            log=f"{self.data.index.max()}:"
            for i in stack[1:len(stack)]:
                aux=i[1].split('\\')[-1]
                log+=f"{aux} {i[2]}, {i[3]} > "
            log=log[0:-2]
            log+=F": {message}"
        else:
            log=f"{self.data.index.max()}:"
            info=stack[1]
            file, line, func = info[1:4]
            aux=file.split('\\')[-1]
            log+=f"{aux} {line}, {func} : {message}"
        with open("./log/"+self.filename+".txt","a") as f:
            f.write(f"{log}\n")
            

    def log(self, message):
        stack = inspect.stack()
        #here = stack[1]
        if len(stack)>2:
            log=f"{self.data.index.max()}:"
            for i in stack[1:len(stack)]:
                aux=i[1].split('\\')[-1]
                log+=f"{aux} {i[2]}, {i[3]} > "
            log=log[0:-2]
            log+=F": {message}"
        else:
            log=f"{self.data.index.max()}:"
            info=stack[1]
            file, line, func = info[1:4]
            aux=file.split('\\')[-1]
            log+=f"{aux} {line}, {func} : {message}"
        with open("./log/"+self.filename+".txt","a") as f:
            f.write(f"{log}\n")
        
    def wait_till_init(self):
        while len(self.data)<1:
            time.sleep(0.1)


    def error(self, message):
        self.stop_logging()
        error = traceback.format_exc()
        with open("./log/"+self.filename+"error.txt","a") as f:
            f.write(f"{message}\n")
        raise CustomError(f"{message}")


if __name__ == '__main__':
    endopoint=Logger()
    endopoint.print("hola")
