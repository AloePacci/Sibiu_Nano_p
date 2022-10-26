from dronekit import connect, VehicleMode, LocationGlobal
import pymavlink
import numpy as np
import math
import traceback
import json
import requests
from pymavlink.dialects.v20 import ardupilotmega as mavlink2 
from pymavlink.quaternion import QuaternionBase
import os
os.environ["MAVLINK20"] = "1"
import time
from logger import Logger
import threading
from sensors import Sensor

#this class downloads gps data from Waterlinked
class GPS_handler():
    def __init__(self, url="https://demo.waterlinked.com", logger=None, antenna_position=None, depth = None):     
        self.url=url
        if logger is None:
            self.log=Logger()
        else:
            self.log=logger   
        self.__close__=False

        #create thread to read GPS
        self.gps_thread=threading.Thread(target=self.read_gps_thread)
        self.gps_thread.start()
        
    def get_data(self, url):
        try:
            r = requests.get(url)
        except requests.exceptions.RequestException as exc:
            self.log.log("Exception occured {}".format(exc))
            return None

        if r.status_code != requests.codes.ok:
            self.log.log("Got error {}: {}".format(r.status_code, r.text))
            return None

        return r.json()


    def get_antenna_position(self):
        return self.get_data(f"{self.url}/api/v1/config/antenna")

    def get_acoustic_position(self):
        return self.get_data(f"{self.url}/api/v1/position/acoustic/filtered")

    def get_global_position(self, acoustic_depth = None):
        return self.get_data(f"{self.url}/api/v1/position/global")

    def close(self):
        self.__close__=True

    def read_gps_thread(self):
        while not self.__close__:
            acoustic_position = self.get_acoustic_position()
            global_position = self.get_global_position()
            self.depth = acoustic_position["z"]
            self.x=acoustic_position['x']
            self.y=acoustic_position['y']
            self.lat=global_position['lat']
            self.lon=global_position['lon']
            self.hdop=global_position['hdop']
            time.sleep(0.05) #limit refresh rate
        self.log.log("gps log closed")

class Message_sender(threading.Thread):
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

    def __init__(self, logger=None, ip="192.168.2.1", port="8003", timeout=6.0, source_system=1, source_component=1):     
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
        self.gps=GPS_handler(logger=self.log)
        self.sensor=Sensor()
        self.__stop__=False
        #create thread to send GPS data

        self.gps_thread=threading.Thread(target=self.send_GPS_Mavlink)
        self.gps_thread.start()



    def goto_deep(self, deepness):
        deep=-int(abs(deepness))
        #self.log.log(f"asked to go to deep {-deep}")
        msg=mavlink2.MAVLink_set_position_target_global_int_message(
            0, 0, 0,  # time (not used), target system, target component
            0, #MAV_FRAME_GLOBAL
            0b110111111000,  # yaw_rate,yaw,unused,Az,Ay,Ax,Vz,Vy,Vx,Z,Y,X
            0,  # lat e7    
            0,  # lon e7
            int(deep),
            0,0,0, #vx,vy,vz
            0,0,0, #ax,ay,az
            0,  # yaw
            0)  # yaw rate
        self.vehicle.send_mavlink(msg)

    def orientate(self, desired_yaw):
        #self.log.log("asked to rotate {desired_yaw}")
        msg = mavlink2.MAVLink_set_attitude_target_message(
            0, 0, 0,  # time (not used), target system, target component
            0b00000111,  # attitude, throttle, uns,uns,uns,yawrate,pitchrate,rollrate
            QuaternionBase([math.radians(angle) for angle in (0,0,desired_yaw)]), #quaternion
            0,0,0,0)  # roll rate, pitch rate, yaw rate, thrust
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def conditional_yaw(self, heading, relative=False):
        if relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            pymavlink.mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used
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
        self.close()

    def emergency_close(self):
        self.vehicle.disarm()
        self.vehicle.mode = VehicleMode("MANUAL")
        self.close()

    def close(self):
        self.gps.close()
        self.log.log("comm closed")
        self.__stop__=True

    def send_GPS_Mavlink(self):
        while not self.__stop__: #while we are not told to stop
            try:
                if self.gps.lon!=None:#if we dont have measure, skip
                    msg=mavlink2.MAVLink_gps_input_message(
                    0, 0,  #time_usec, gps_id
                    0b11111100, #flag ingnore[vertical accuracy, hori acc, sped acc, vel vert, vel horiz, vdop, hdop, depth]
                    0,0, #time_week_ms, time_week,
                    3, #0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
                    int(self.gps.lat*1e7),
                    int(self.gps.lon*1e7),
                    self.gps.depth,
                    self.gps.hdop,
                    65535, #uin16 max for Vdop
                    0,0,0, #vn ve vd
                    0,0,0, #speed, hori and vert accuracy
                    9, #satellites_visible, min value 4. actual value 3 from antenna
                    0, #yaw not available
                    )
                    self.vehicle.send_mavlink(msg)
            except:
                pass
            time.sleep(0.1) #5hz is enough, we give more cause yolo

    def calculate_distance(self, lat, lon):
        """
        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """

        # Convert to radians #
        lat1 = np.radians(self.vehicle.location.global_relative_frame.lat)
        lat2 = np.radians(lat)
        lon1 = np.radians(self.vehicle.location.global_relative_frame.lon)
        lon2 = np.radians(lon)

        # Obtains the latitude/longitude differences #
        d_lat = lat2 - lat1
        d_lon = lon2 - lon1

        # Returns True if the waypoint is within 1.5 meters the ASV position
        a = np.sin(0.5 * d_lat) ** 2 + np.sin(0.5 * d_lon) ** 2 * np.cos(lat1) * np.cos(lat2)
        c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))
        return 6378100.0 * c
            
    def goto(self, lat, lon, depth):
        ekf_failed=False #memory for EKF
        ekf_counter=0
        travel_counter=0
        #if we are too far away cancel
        if self.calculate_distance(lat,lon)>100:
            self.log.print("we are too far away from the point")
            return
        self.log.print(f"asked to go to {[lat, lon]}, distance {self.calculate_distance(lat,lon)}")
        #put vehicle in guided mode and ask to go to point
        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1) #wait for mode to change
        self.vehicle.simple_goto(LocationGlobal(lat, lon, -abs(depth)))
        while self.calculate_distance(lat,lon)>1.5 and (depth-self.vehicle.location.global_relative_frame.alt)>1.5: #while we have not reached the point
            #check system is able to go to point
            if not self.vehicle.ekf_ok:
                self.log.print("ekf failed")
                self.vehicle.mode = VehicleMode("ALT_HOLD") #maintain position
                ekf_failed=True
                if ekf_counter%30 == 0: #log each 3 secs
                    self.log.log(f"System Status: \nmode {self.vehicle.mode.name}, GPS_status: {self.vehicle.gps_0}, System status: {self.vehicle.system_status.state}, System able to arm {self.vehicle.is_armable} ")
                ekf_counter+=1
            
            elif ekf_failed: #if system failed, recover
                ekf_failed=False
                self.log.print("EKF failsafe cleared, resuming mission")
                self.log.log(f"System Status: mode {self.vehicle.mode.name}, GPS_status: {self.vehicle.gps_0}, System status: {self.vehicle.system_status.state}, System able to arm {self.vehicle.is_armable} ")
                ekf_failed=False #reset flag
                ekf_counter=0
                self.vehicle.arm() #arm to avoid inconsistent state
                self.vehicle.mode = VehicleMode("GUIDED") #restore mode
                self.vehicle.simple_goto(LocationGlobal(lat, lon, -abs(depth))) #restore point
            if travelling_counter%30 == 0: #each 3 seconds
                self.log.print(f"distance [{self.calculate_distance(lat,lon)},{(depth-self.vehicle.location.global_relative_frame.alt)}]")
            
            travelling_counter+=1
            time.sleep(0.1)
        # after reaching samplepoint
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        self.sensor.take_sample()



if __name__ == '__main__':
    endopoint=Message_sender()