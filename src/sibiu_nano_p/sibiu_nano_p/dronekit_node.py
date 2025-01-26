import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
#TODO: deprecate dronekit, use pymavlink
from dronekit import connect, VehicleMode, LocationGlobal
import numpy as np
import pymavlink
import traceback
import math
from math import sqrt, pow, atan2
from .submodulos.dictionary import dictionary
import time
from pymavlink.dialects.v20 import ardupilotmega as mavlink2 #for obstacle distance information
from pymavlink.quaternion import QuaternionBase
import threading
from numpy import uint
#import intefaces
from uuv_interfaces.msg import Status, Nodeupdate, Camera, Obstacles, Location
from uuv_interfaces.srv import CommandBool, ASVmode, Newpoint, Takesample, CommandStr, CommandInt, RCOverride
from uuv_interfaces.action import Goto, SensorSample


import os
os.environ["MAVLINK20"] = "1"

# This node generates the necessary services for  comunication towards the drone
#parameters are only read at start

#we define a simple PID
class pid:
    def __init__(self, kp, ki, kd, sat):
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.dererr=0.0
        self.interr=0.0
        self.sat=sat

    def derivative(self, error):
        a=self.dererr
        self.dererr=error
        return self.kd*(error-a)

    def integral(self, error):
        if self.sat==0 or (abs(self.proportional(error)+self.interr * self.ki)<self.sat):
            self.interr+=error
        return self.interr * self.ki

    def proportional(self, error):
        return error*self.kp

    def PD(self, error):
        return self.proportional(error)+self.derivative(error)
    
    def PI(self, error):
        return self.proportional(error)+self.integral(error)

    def PID(self, error):
        output=self.proportional(error)+self.integral(error)+self.derivative(error)
        if abs(output)<self.sat:
            return int(output)
        if output>self.sat:
            return self.sat
        return -self.sat


#this class downloads gps data from Waterlinked
class GPS_handler():
    def __init__(self, url="http://192.168.7.1", antenna_position=None, depth = None):     
        self.url=url
        if logger is None:
            self.log=Logger()
        else:
            self.log=logger   
        self.__close__=False

        #create thread to read GPS
        self.gps_thread=threading.Thread(target=self.read_gps_thread)
        self.gps_thread.start()
        self.get_logger().info(f'GPS read started') 
        
    def get_data(self, url):
        try:
            r = requests.get(url)
        except requests.exceptions.RequestException as exc:
            self.get_logger().error("Exception occured {}".format(exc))
            return None

        if r.status_code != requests.codes.ok:
            self.get_logger().error("Got error {}: {}".format(r.status_code, r.text))
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
            try:
                    #if acoustic_position["position_valid"]==True:
                    self.depth = acoustic_position["z"]
                    self.x=acoustic_position['x']
                    self.y=acoustic_position['y']
                    self.lat=global_position['lat']
                    self.lon=global_position['lon']
                    self.hdop=global_position['hdop']
                    #else:
                    #   print(f"invalid_pos {[acoustic_position['x'], acoustic_position['y']]}")
            except:
                pass
            time.sleep(0.05) #limit refresh rate
        self.get_logger().info("gps log closed")

class Dronekit_node(Node):


    #TODO define a wildcard_caldback for dronekit criticals
    """
       def wildcard_callback(self, attr_name, value):
           self.get_logger().fatal(f"dronekit error {attr_name}\n value: {value}")
       self.vehicle.add_attribute_listener('*', wildcard_callback)
    """


    #his functions defines and assigns value to the parameters
    def parameters(self):
        self.declare_parameter('vehicle_ip', '192.168.2.1:8002')
        self.vehicle_ip = self.get_parameter('vehicle_ip').get_parameter_value().string_value
        self.declare_parameter('connect_timeout', 15)
        self.connect_timeout = self.get_parameter('connect_timeout').get_parameter_value().integer_value
        self.declare_parameter('max_distance', 100)
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().integer_value
        self.declare_parameter('vehicle_id', 1)
        self.vehicle_id=self.get_parameter('vehicle_id').get_parameter_value().integer_value
        self.declare_parameter('arm_timeout', 15)
        self.arm_timeout = self.get_parameter('arm_timeout').get_parameter_value().integer_value
        self.declare_parameter('travelling_timeout', 60)
        self.travelling_timeout = self.get_parameter('travelling_timeout').get_parameter_value().integer_value
        self.declare_parameter('override_control', False)
        self.override_control = self.get_parameter('travelling_timeout').get_parameter_value().bool_value


    #this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        #host
        self.arm_vehicle_service = self.create_service(CommandBool, 'arm_vehicle', self.arm_vehicle_callback)
        self.change_ASV_mode_service = self.create_service(ASVmode, 'change_asv_mode', self.change_asv_mode_callback)
        self.reset_home_service = self.create_service(CommandBool, 'reset_home', self.reset_home_callback)
        self.goto_deep_service = self.create_service(Newpoint, 'goto_deep', self.goto_deep_callback)
        self.orientate_service = self.create_service(Newpoint, 'orientate', self.orientate_callback)
        self.override_service = self.create_service(RCOverride, 'override_RC', self.override_RC_callback)
        self.set_sysid_service = self.create_service(CommandInt, 'set_sysid', self.set_sys_id_callback)
        self.close_service = self.create_service(CommandBool, 'close', self.close_callback)
        self.playtune_service = self.create_service(CommandStr, 'playtune', self.play_tune_callback)

        #client
        self.asv_mission_mode_client = self.create_client(ASVmode, 'change_mission_mode')
        self.take_sample_client = self.create_client(Takesample, 'get_sample')
        self.calculate_path_client = self.create_client(Newpoint, 'calculate_path')


    def declare_topics(self):
        timer_period = 0.5  # seconds
        self.status_publisher = self.create_publisher(Status, 'status', 10)
        self.status_publisher_timer = self.create_timer(timer_period, self.status_publish)
        self.obstacles_subscriber = self.create_subscription(Obstacles, 'camera_obstacles', self.camera_obstacles_callback, 10)
        self.waypoints_publisher = self.create_publisher(Location, 'waypoint_mark', 10)


    def declare_actions(self):
        self.go_to_server = ActionServer(self, Goto, 'goto', execute_callback=self.goto_execute_callback,
                                         goal_callback=self.goto_accept,
                                         handle_accepted_callback=self.goto_accepted_callback,
                                         cancel_callback=self.goto_cancel,
                                         callback_group=ReentrantCallbackGroup())
        self.goto_goal_handle = None
        #TODO: if we want to call this server more than once at a time consider using multithread executor in sping and reentrant_callback_group in action servers
        self.sensor_action_client = ActionClient(self, SensorSample, 'sensor_read') #action client

    def __init__(self):
        # start the node
        super().__init__('Dronekit_node')

        # declare parameters
        self.parameters()
        self.status = Status()

        self.last_obstacle_distance_sent_ms=0
        # connect to vehicle
        self.get_logger().info(f"Connecting to vehicle in {self.vehicle_ip}")
        try:
            self.vehicle = connect(self.vehicle_ip, timeout=self.connect_timeout, source_system=1, source_component=93)
            self.dictionary()
            self.declare_topics()
            self.declare_services()
            self.declare_actions()
            self.cmds = self.vehicle.commands
            self.get_logger().info(f"Waiting for initialization")
            self.cmds.download()
            self.cmds.wait_ready()
            self.home = self.vehicle.home_location
            self.get_logger().info(f"Vehicle home location is {self.home}")
        except ConnectionRefusedError:
            self.get_logger().error(f"Connection to UUV was refused")
            self.get_logger().fatal("Drone module is dead")
            self.destroy_node()
            return
        except OSError:
            self.get_logger().error(f"UUV was not found in the same network")
            self.get_logger().fatal("Drone module is dead")
            self.destroy_node()
            return
        except TimeoutError:
            self.get_logger().error(f"UUV port was busy, timeout error")
            self.get_logger().fatal("Drone module is dead")
            self.destroy_node()
            return
        except:
            error = traceback.format_exc()
            self.get_logger().error(f"Connection to UUV could not be made, unknown error:\n {error}")
            self.get_logger().fatal("Drone module is dead")
            self.destroy_node()
            return
    self.get_logger().info(f"Connection SUCCESS")
        self.gps=GPS_handler(logger=self.log)
        self.sensor=Sensor()
        self.__stop__=False
        #create thread to send GPS data

        self.gps_thread=threading.Thread(target=self.send_GPS_Mavlink)
        self.gps_thread.start()

        if self.override_control:
            self.get_logger().info(f'control type override') 
            self.override([0,0,0,0,0,0]) #override values
            self.control_thread=threading.Thread(target=self.override_iteration)
            self.control_thread.start()
            #instanciate PID
            self.x_axis=pid(100, 0.01, 5, 500)
            self.y_axis=pid(100, 0.01, 5, 500) 
            self.z_axis=pid(150, 1, 60, 500) 
            self.roll_axis=pid(100, 0, 400, 500)
            self.pitch_axis=pid(100, 0, 400, 500) 
            self.yaw_axis=pid(0.8, 0.02 , 0.1, 200)

            self.last=[] #last values for detecting changes reducing overhead
        

    def arm_vehicle_callback(self, request, response):
        """
        Arming vehicle function.
        Args:
             request.value:
                -True : arm vehicle
                -False: disarm vehicle
        """
        if self.status.manual_mode:
            self.get_logger().info(f"RC in manual, cannot arm or disarm")
            return response
        try:
            if request.value:
                self.vehicle.arm()
                self.get_logger().info('Vehicle armed')
                response.success=True
            else:
                self.vehicle.disarm()
                self.get_logger().info('Vehicle disarmed')
                response.success=True
        except:
            self.get_logger().error('Couldn\'t arm vehicle')
            response.success=False
        return response

    def status_publish(self):
        try:
            # TODO: es necesario que dronekit te devuelva valores no corruptos, crash otherwise
            self.status.lat = self.vehicle.location.global_relative_frame.lat
            self.status.lon = self.vehicle.location.global_relative_frame.lon
            self.status.yaw = self.vehicle.attitude.yaw
        except:
            pass
        #Treat batery apart, as it uses to fail
        try:
            self.status.battery = float(self.vehicle.battery.voltage)
        except:
            pass
        #finally if one of these values fails, dont send the message, this values are always reported
        try:
            self.status.armed = self.vehicle.armed
            self.status.vehicle_id = self.vehicle_id
            self.status.asv_mode=str(self.vehicle.mode.name)
            self.status.ekf_ok = bool(self.vehicle.ekf_ok)
            self.status_publisher.publish(self.status)
        except:
            pass
        


    def get_bearing(self, location1, location2):
        """
        Returns the bearing between the two LocationGlobal objects passed as parameters.
        This method is an approximation, and may not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py`
        Args:
            location1: Actual position (`dronekit.LocationGlobal`).
            location2: Reference position (`dronekit.LocationGlobal).
        Returns:
            The angle difference from `location1 to `location2
        """
        off_x = location2.lon - location1.lon
        off_y = location2.lat - location1.lat
        bearing = 90.00 + atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing


    def condition_yaw(self, heading, relative=False):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.
        By default the yaw of the vehicle will follow the direction of travel. After setting
        the yaw using this function there is no way to return to the default yaw "follow direction
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
        For more information see:
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        """
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

    def reached_position(self, goal_loc):
        """
        Args:
            goal_loc: Reference position (dronekit.LocationGlobal).
        Returns:
            'True' if the UUV distance respecto to the target Waypoint is less than 0.5 meters.
        """

        return self.calculate_distance(goal_loc) < 1.5

    def calculate_distance(self, goal_loc):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        Args:
            goal_loc: Reference position (dronekit.LocationGlobal).
        Returns:
            distance from the UUV to the goal_loc in meters
        """

        # Convert to radians #
        lat1 = np.radians(self.vehicle.location.global_relative_frame.lat)
        lat2 = np.radians(goal_loc.lat)
        lon1 = np.radians(self.vehicle.location.global_relative_frame.lon)
        lon2 = np.radians(goal_loc.lon)

        # Obtains the latitude/longitude differences #
        d_lat = lat2 - lat1
        d_lon = lon2 - lon1

        # Returns True if the waypoint is within 1.5 meters the UUV position
        a = np.sin(0.5 * d_lat) ** 2 + np.sin(0.5 * d_lon) ** 2 * np.cos(lat1) * np.cos(lat2)
        c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))
        return 6378100.0 * c

    def change_asv_mode_callback(self, request, response):
        #string takes preference before int
        if self.status.manual_mode:
            self.get_logger().info(f"RC in manual, cannot change mode")
            return response
        if len(request.asv_mode_str) != 0:
            if request.asv_mode_str in self.mode_type.values():
                mode=request.asv_mode_str
            else:
                self.get_logger().info(f"{request.asv_mode_str} is not a valid mode")
                response.success=False
                return response
        else:
            if request.asv_mode in self.mode_type:
                mode=self.mode_type[str(request.asv_mode)]
            else:
                self.get_logger().info(f"{request.asv_mode} is not a valid mode")
                response.success = False
                return response
        self.vehicle.mode = VehicleMode(mode)
        response.success=True
        return response
        #TODO: add other responses

    ######################################### ACTION GOTO DESCRIPTION ############################################

    def goto_accept(self, goal_request):
        self.get_logger().info(f'Action call received')
        #if we are attending another call exit
        if self.goto_goal_handle is not None and self.goto_goal_handle.is_active:
            self.get_logger().error(f'Action is busy')
            return GoalResponse.REJECT
        if not self.vehicle.armed:
            self.get_logger().error(f'Error: vehicle should be armed. Vehicle is in {self.vehicle.mode}.')
            if not self.vehicle.ekf_ok:
                self.get_logger().error(f"EKF fail, System Status: mode {self.vehicle.mode.name}, GPS_status: {self.vehicle.gps_0}, System status: {self.vehicle.system_status.state}, System able to arm {self.vehicle.is_armable}")
        self.get_logger().info(f'Action accepted')
        return GoalResponse.ACCEPT

    def goto_accepted_callback(self, goal_handle):
        #we could make a list of goal handles and make a queue of points to go to and accept everything,
        #for the time being, we will just start executing
        self.goto_goal_handle=goal_handle
        if self.calculate_distance(goal_handle.request.samplepoint)>self.max_distance:
            self.get_logger().warning(f'we are too far away from the destination point {self.calculate_distance(goal_handle.request.samplepoint)}m, cancelling action')
            result.finish_flag = "Point too far away"
            goal_handle.abort()
            return Goto.Result()
        goal_handle.execute() #execute goto_execute_callback

    def goto_cancel(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


    def goto_execute_callback(self, goal_handle): #TODO: check correct behaviour
        feedback_msg = Goto.Feedback()
        result=Goto.Result()
        arm_counter=0 #counter to leave if several seconds disarmed
        ekf_counter=0 #counter to leave if several seconds with ekf in bad state
        ekf_failed=False #bool to know were we come from

        #Calculate Path
        destination=Newpoint.Request()
        destination.new_point=goal_handle.request.samplepoint
        path=self.call_service(self.calculate_path_client,destination)
        #if there is no path, go directly to destination
        if path==False or path.success==False:
            self.get_logger().error("something went wrong with path planner")
            #result.finish_flag = "Planner couldn't calculate point"
            #goal_handle.abort()
            #return result
            self.get_logger().info("going to point without planner")
            path=[goal_handle.request.samplepoint] #TODO: decide whether to go directly to point or cancel action
        else:
            #extract path
            path=path.point_list
        self.vehicle.mode = VehicleMode("GUIDED")
        #self.condition_yaw(self.get_bearing(self.vehicle.location.global_relative_frame, goal_handle.request.samplepoint))
        time.sleep(1)
        while (rclpy.ok()) and (len(path)>0):
            travelling_counter=0 #counter time since going to waypoint
            self.vehicle.simple_goto(LocationGlobal(path[0].lat, path[0].lon, 0.0))
            if len(path)>1:
                location1= Location()
                location1.lat=path[0].lat
                location1.lon=path[0].lon
                self.waypoints_publisher.publish(location1)
            while rclpy.ok() and not self.reached_position(path[0]): #while we reach the goal, check for events
                if goal_handle.is_cancel_requested:#event mission canceled
                    #make it loiter around actual position
                    goal_handle.canceled()
                    self.vehicle.mode = VehicleMode("ALT_HOLD")
                    self.get_logger().info('Goto Action canceled')
                    result.finish_flag = "Action cancelled"
                    return result
                
                elif self.status.manual_mode:
                        self.get_logger().warning("manual interruption during mission")
                        result.finish_flag = "Manual interruption"
                        goal_handle.abort()
                        return result

                elif not self.vehicle.ekf_ok: #if ekf is not ok, stop
                    if self.vehicle.mode != VehicleMode("MANUAL"):
                        self.get_logger().warning('EKF FAILED, sensor read not good enough, waiting for better signal, switching to manual and disarming')
                        self.vehicle.mode = VehicleMode("MANUAL")
                        self.vehicle.disarm()
                        ekf_failed=True
                    elif ekf_counter%30 == 0:
                            self.get_logger().info(f"System Status: \nmode {self.vehicle.mode.name}, GPS_status: {self.vehicle.gps_0}, System status: {self.vehicle.system_status.state}, System able to arm {self.vehicle.is_armable} ")
                    if ekf_counter > self.ekf_timeout*10:
                        result.finish_flag = "ekf timeout"
                        self.get_logger().info('EKF timeout, exiting action')
                        goal_handle.abort()
                        return result
                    ekf_counter+=1
                
                elif self.vehicle.mode != VehicleMode("GUIDED"): #vehicle is not in desired mode
                    self.get_logger().warning("uuv_mode was changed externally")
                    if ekf_failed:
                        self.get_logger().info("EKF failsafe cleared, resuming mission")
                        self.get_logger().info(f"System Status: mode {self.vehicle.mode.name}, GPS_status: {self.vehicle.gps_0}, System status: {self.vehicle.system_status.state}, System able to arm {self.vehicle.is_armable} ")
                        ekf_failed=False #reset flag
                        ekf_counter=0
                        self.vehicle.arm() #arm to avoid inconsistent state
                    self.vehicle.mode = VehicleMode("GUIDED") #restore mode
                    self.condition_yaw(self.get_bearing(self.vehicle.location.global_relative_frame, goal_handle.request.samplepoint))
                    self.vehicle.simple_goto(LocationGlobal(goal_handle.request.samplepoint.lat, goal_handle.request.samplepoint.lon, 0.0))
            
                elif not self.status.armed: #event vehicle disarmed
                    if not self.vehicle.is_armable:
                        self.get_logger().fatal('vehicle is not armable, inconsistent state')
                    else:
                        if arm_counter%10==0:
                            self.get_logger().info('received external disarm in auto mode, waiting for RC')
                        if self.vehicle.channels['5']>1200:
                            self.vehicle.arm()
                            self.get_logger().info('vehicle armed')
                            arm_counter=0
                        else:
                            arm_counter+=1
                        if arm_counter > self.arm_timeout*10:
                            self.get_logger().info('Waited for RC for too long, exiting action')
                            result.finish_flag = "timeout vehicle disarmed"
                            goal_handle.abort()
                            return result

                if travelling_counter>self.travelling_timeout*10:
                    self.get_logger().info('timeout reaching the point, exiting action')
                    result.finish_flag = "timeout reaching point"
                    goal_handle.abort()
                    return result
                
                if travelling_counter%30 == 0: #each 3 seconds
                    feedback_msg.distance = self.calculate_distance(path[0])
                    goal_handle.publish_feedback(feedback_msg)

                travelling_counter+=1
                time.sleep(0.1)
            #waypoint reached so go to next one
            if len(path)>1:
                self.get_logger().info(f"waypoint reached going to [{path[1].lat},{path[1].lon}")
            path.pop(0)
        # after reaching samplepoint
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        goal_handle.succeed()
        self.get_logger().info('Goal reached, waiting for sample')
        self.waiting_for_sensor_read=True
        self.sensor_action_client.wait_for_server()
        goal_msg = SensorSample.Goal()
        self._send_goal_future = self.sensor_action_client.send_goal_async(goal_msg, feedback_callback=self.sensor_feedback_callback)
        self._send_goal_future.add_done_callback(self.sensor_response)
        while rclpy.ok() and self.waiting_for_sensor_read:
            if goal_handle.is_cancel_requested:#event mission canceled
                    goal_handle.canceled() #acknowledge goto cancelled
                    self.get_logger().info('Goto Action canceled')
                    result.finish_flag = "Action cancelled"
                    self.cancel_sensor_read() #stop sensor read
                    return result
                
            elif self.status.manual_mode:
                    self.get_logger().warning("manual interruption during mission")
                    result.finish_flag = "Manual interruption"
                    self.cancel_sensor_read()
                    goal_handle.abort()
                    return result
        #self.get_logger().debug(f'Sample value {value}')
        result.success = True
        result.finish_flag = "Normal execution"
        return result



    ####################################### END ACTION DEFINITION ############################################



    def call_service(self, client,  msg):
        # TODO: raise error to avoid infinite wait if service is not up, after all means a module is not active $$ watchdog will be in charge
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info(f'service {client.srv_name} not available', once=True)
            return False
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    return response
                break

    def dictionary(self):
        self.mode_type=dictionary("ASV_MODE")
        
    
    """
        This function forces the stop of a sensor read
    """   
    def cancel_sensor_read(self):
        if self.waiting_for_sensor_read: #if the action was active
            self.get_logger().info("the uuv was taking a sample, canceling")
            self.sensor_goal_handle.cancel_goal_async()
        pass


    """
        This function prints the feedback from the goto action
    """     
    def sensor_feedback_callback(self, feedback):
        sensor_read= feedback.feedback.sensor_read #TODO: unused

    """
        This function is the first answer from the action service, indicates whether the request is accepted or rejected
    """   
    def sensor_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Sensor read rejected :(, something is not working')
            return
        self.get_logger().debug('Sensor read accepted :)')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.go_to_finished)


    """
        This function prints the result once the action finishes
    """   
    def sensor_read_finished(self, future):
        result = future.result().result #this variable is the result specified in the action
        status = future.result().status # ABORT = 4     CANCELED = 5    CANCEL_GOAL = 2     EXECUTE = 1     SUCCEED = 3     NOT_SPECIFIED = 6
        status_string= ["0 IS NOT AN STATE", "EXECUTE", "CANCEL_GOAL", "SUCCEEDED", "ABORT", "CANCELED", "NOT SPECIFIED"]
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().debug('Goal succeeded! Result: {0}'.format(result.success))
        else:
            self.get_logger().info(f'Sensor read failed with status: {status_string[int(status)]}')
            self.get_logger().info(f'reason : {result.finish_flag}')           
        self.waiting_for_sensor_read = False # we indicate that sensor read has finished



    #TODO: Dronekit is not able to send this message


    """
    This function is called as a callback from the topic obstacles
    @params distance, angle_increment:   arrays indicating in each index distance and angle occupied by an obstacle.
    if distance is greater than 20m then it is considered no obstacle. maximum ammount inpts 72.
    
    the data received should be a mean of the distance towards the object in order to achieve less than 72 obstacles in a 180º fov.
    check https://ardupilot.org/dev/docs/code-overview-object-avoidance.html
    """
    def camera_obstacles_callback(self, msg):

        angle_increment=msg.angle_increment

        distance=[2000 for i in range(72)] #2000 or greater is no obstacle

        if len(msg.distance)>72: #max size of array is 72
            size=72
        else:
            size=len(msg.distance)

        for i in range(size):
            distance[i]=uint(msg.distance[i])


        msg2 = mavlink2.MAVLink_obstacle_distance_message(
            0,                      # us Timestamp (UNIX time or time since system boot)
            3,                      # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
            distance,               # distances,    uint16_t[72],   cm
            int(3),                 # increment,    uint8_t,        deg
            int(30),	            # min_distance, uint16_t,       cm
            int(2000),              # max_distance, uint16_t,       cm
            float(angle_increment),	            # increment_f,  float,          deg #https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
            float(-54), # angle_offset, float,          deg
            12                      # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
            )

        self.get_logger().info(f"{msg2}",once=True)
        try:
            self.vehicle.send_mavlink(msg2)
        except:
            self.get_logger().info(f'obstacle send failed:{traceback.format_exc()}')    


    def reset_home_callback(self, request, response):
        self.vehicle.home_location = self.vehicle.location.global_frame
        response.success=True
        return response



    def goto_deep_callback(self, request, response):
        self.get_logger().info(f'going to deep: {request.new_point.deepness}') 
        deep=-int(abs(request.new_point.deepness))
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
        response.success=True
        return response

    def orientate_callback(self, request, response):
        self.get_logger().info(f'rotating to: {request.new_point.orientation}') 
        msg = mavlink2.MAVLink_set_attitude_target_message(
            0, 0, 0,  # time (not used), target system, target component
            0b00000111,  # attitude, throttle, uns,uns,uns,yawrate,pitchrate,rollrate
            QuaternionBase([math.radians(angle) for angle in (0,0,request.new_point.orientation)]), #quaternion
            0,0,0,0)  # roll rate, pitch rate, yaw rate, thrust
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        response.success=True
        return response
        
    def play_tune_callback(self, request, response):
        self.get_logger().info(f"asked to play tune {request.string}")
        msg=mavlink2.MAVLink_play_tune_message(
        0, 0,  #target system, target component
        bytes('', 'utf-8'), #format
        bytes('', 'utf-8') #tune
        )
        self.vehicle.send_mavlink(msg)
        response.success=True
        return response

    def close_callback(self, request, response):
        self.gps.close()
        self.get_logger().info("comm closed")
        self.vehicle.disarm()
        self.vehicle.mode = VehicleMode("MANUAL")
        self.vehicle.close()
        self.__stop__=True
        response.success=True
        return response

    def override_iteration(self):
        try:
            errorx=self.override["x"]-self.gps.x
            errory=self.override["y"]-self.gps.y
        except:
            errorx=0
            errory=0
        errorz=self.override["z"]-self.vehicle.location.global_frame.alt
        if self.last==[errorx,errory,errorz,self.vehicle.attitude.roll,self.vehicle.attitude.pitch,self.vehicle.heading]: #avoid unnecesary iterations
            return 
        else:
            self.last=[errorx,errory,errorz,self.vehicle.attitude.roll,self.vehicle.attitude.pitch,self.vehicle.heading]
        
        #for heading
        #force simetry
        vehicleheading=self.vehicle.heading
        if vehicleheading>180:
            vehicleheading=360-vehicleheading
        if abs(heading-vehicleheading) > 180: #we choose the shortest path
                    if heading>0:
                        heading-=360
                    else:
                        heading+=360
        rolsignal=self.roll_axis.PID(-self.vehicle.attitude.roll)
        pitchsignal=self.pitch_axis.PID(self.vehicle.attitude.pitch)
        yawsignal=self.yaw_axis.PID(self.override["yaw"]-vehicleheading)
        angulo_ataque=self.vehicle.heading
        distance=sqrt(pow(errorx,2)+pow(errory,2))
        xvalue=int(self.x_axis.PID(distance)*math.sin(angulo_ataque))
        yvalue=int(self.y_axis.PID(distance)*math.cos(angulo_ataque))
        zvalue=self.z_axis.PID(errorz)


    def set_sys_id_callback(self, request, response):
        #self.vehicle.parameters['SYSID_MYGCS']=request.value
        response.success=True
        return response
        

    
    def override_RC_callback(self, request, response):
        self.override={"x":request.x,"y":request.y,"z":request.z,"roll":request.roll,"pitch":request.pitch,"yaw":request.yaw}
        response.success=True
        return response


def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    try:
        #start a class that servers the services
        dronekit_node = Dronekit_node()
        #loop the node
        rclpy.spin(dronekit_node, executor=MultiThreadedExecutor())

        dronekit_node.destroy_node()

    except:
        """
        There has been an error with the program, so we will send the error log to the watchdog
        """
        x = rclpy.create_node('dronekit_node') #we state what node we are
        publisher = x.create_publisher(Nodeupdate, '_internal_error', 10) #we create the publisher
        #we create the message
        msg = Nodeupdate()
        msg.node = "dronekit_node" #our identity
        msg.message = traceback.format_exc() #the error
        #to be sure the message reaches, we must wait till watchdog is listening (publisher needs time to start up)
        #TODO: Vulnerable si alguien esta haciendo echo del topic, el unico subscriptor debe ser wathdog
        # este topic está oculto en echo al usar _
        while publisher.get_subscription_count() == 0: #while no one is listening
            time.sleep(0.01) #we wait
        publisher.publish(msg) #we send the message
        x.get_logger().fatal(msg.message)
        x.destroy_node() #we destroy node and finish
    #after close connection shut down ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
