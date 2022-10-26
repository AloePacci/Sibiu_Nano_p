#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Wrench
from sensor_msgs.msg import FluidPressure ,MagneticField, Imu, NavSatFix
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, atan, cos, sin, tan, pi
import tf.transformations
from visualization_msgs.msg import Marker
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
import sys
from sibiu_nano_p_control.srv import *



#global variables

#parameters
ns=rospy.get_param('/controlador/namespace', '/sibiu_nano_p') #namespace
debug=int(rospy.get_param('/controlador/debug_level', 2)) #nivel de debug
tolerance=float(rospy.get_param('/controlador/tolerance', 0.1)) #tolerancia enviar respuesta servicios
tolerance_angular=float(rospy.get_param('/controlador/tolerance_angular', 0.1)) #tolerancia angular

#input parameter



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
            return output
        if output>self.sat:
            return self.sat
        return -self.sat

#this function translates the debug level from number to value
def debug_level(number):
    if number==0:
        return rospy.NONE
    if number==1:
        return rospy.FATAL
    if number==2:
        return rospy.ERROR
    if number==3:
        return rospy.INFO
    if number==4:
        return rospy.WARN
    if number==5:
        return rospy.DEBUG

class controller:
    def __init__(self):
        rospy.init_node('control_publisher', anonymous=True, log_level=debug_level(debug)) # we init the node
        #we subscribe to the sensors
        self.pressure_sub=rospy.Subscriber(ns+'/pressure', FluidPressure, self.update_pressure)
        self.compass_sub=rospy.Subscriber(ns+'/magnetometer', MagneticField, self.update_magentometer)
        self.IMU_sub=rospy.Subscriber(ns+'/imu', Imu, self.update_imu)
        self.GPS_sub=rospy.Subscriber(ns+'/gps', NavSatFix, self.update_GPS)
        self.pose_sub=rospy.Subscriber(ns+'/pose_gt', Odometry, self.update_pose)
        
        #we prepare the publisher for the control allocation
        self.thruster_manager_pub=rospy.Publisher(ns+'/thruster_manager/input', Wrench, queue_size=10)

        #we prepare the publisher to plot the reference
        self.reference_pub=rospy.Publisher('controller/reference', Twist, queue_size=10)

        #we create the services for the planner
        self.altitude_service = rospy.Service('/controller/'+'_reference_altitude', altitude, self.height_state)
        self.position_service = rospy.Service('/controller/reference_position', position, self.position_state)
        self.horizontal_plane_service = rospy.Service('/controller/'+'_reference_horizontal', horizontal_plane, self.horizontal_plane_state)
        self.heading_service = rospy.Service('/controller/'+'_reference_heading', heading, self.heading_state)


        #we define one PID for each DOF
        self.x_axis=pid(180, 0.01, 20, 140)
        self.y_axis=pid(180, 0.01, 20, 140) 
        self.z_axis=pid(620, 7, 320, 6500) 
        self.roll_axis=pid(5, 0, 150, 1500)
        self.pitch_axis=pid(5, 0, 150, 1500) 
        self.yaw_axis=pid(23, 0.005 , 11, 1500)
        
        #State variables to deactivate/activate control when needed
        self.control_buoyancy=False
        self.control_horizontal=False
        self.control_heading=False

        #variables we will make use of
        self.rate = rospy.Rate(10) #10Hz
        self.signal=Wrench() # output signal
        self.rate.sleep() #we wait until the sensors give the first value
        self.reference=Twist()
        self.last_reference=Twist()
        self.control()
        

    def update_pose(self, data):
        self.pose=data.pose.pose.position
        orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        self.rot=tf.transformations.euler_from_quaternion(orientation) #roll pitch yaw



    def update_pressure(self, data):
        self.pressure=data
        self.deepness=(self.pressure.fluid_pressure-101.325)*10/103.25 #transformation between pressure and deepness

    def update_magentometer(self, data):
        self.magnetic_field=data.magnetic_field

    def update_imu(self, data):
        self.imu_angular=data.angular_velocity
        self.imu_linear=data.linear_acceleration
        orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (self.roll,self.pitch,self.yaw) = tf.transformations.euler_from_quaternion(orientation)


    def update_GPS(self, data):
        self.gps=[data.latitude, data.longitude, data.altitude]


    def control(self):
        while True:

            #roll control
            self.signal.torque.x=self.roll_axis.PID(self.roll)

            #pitch control
            self.signal.torque.y=self.pitch_axis.PID(-self.pitch) #negative due to axis of submarine vs axis IMU
            
            #horizontal control
            if self.control_horizontal==True:
                angulo_ataque=atan2(self.reference.linear.y-self.pose.y,self.reference.linear.x-self.pose.x)-self.yaw
                distance=sqrt(pow(self.reference.linear.x-self.pose.x,2)+pow(self.reference.linear.y-self.pose.y,2))
                self.signal.force.x=self.x_axis.PID(distance)*sin(angulo_ataque)
                self.signal.force.y=self.y_axis.PID(distance)*cos(angulo_ataque)
            
            
            #heigth control
            if self.control_buoyancy==True:
                self.signal.force.z=self.z_axis.PID(self.reference.linear.z-self.deepness)
            

            #heading control
            if self.control_heading==True:
                if abs(self.reference.angular.z-self.yaw) > pi: #we choose the shortest path
                    if self.reference.angular.z>0:
                        self.reference.angular.z-=2*pi
                    else:
                        self.reference.angular.z+=2*pi
                self.signal.torque.z=self.yaw_axis.PID(self.reference.angular.z-self.yaw)

            self.thruster_manager_pub.publish(self.signal) #we publish the control output to the control allocator
            self.rate.sleep() 
            if self.last_reference.linear.x != self.reference.linear.x or self.last_reference.linear.y != self.reference.linear.y or self.last_reference.linear.z != self.reference.linear.z or self.last_reference.angular.z != self.reference.angular.z:
                self.reference_pub.publish(self.reference) #we send the reference to plot
                self.last_reference.linear.x=self.reference.linear.x
                self.last_reference.linear.y=self.reference.linear.y
                self.last_reference.linear.z=self.reference.linear.z
                self.last_reference.angular.z=self.reference.angular.z


            


    def height_state(self, data):
        self.reference.linear.z=data.z #update reference value
        self.control_buoyancy=True#activate control
        while abs(self.reference.linear.z-self.deepness)>tolerance: #stablish control exit
            self.rate.sleep()
        return True
        
    def position_state(self, data):
        self.reference.linear.x=data.x#update reference value
        self.reference.linear.y=data.y
        self.reference.linear.z=data.z
        self.reference.angular.z=data.yaw
        self.control_horizontal=True #activate control
        self.control_buoyancy=True
        self.control_heading=True
        while (sqrt(pow(self.reference.linear.x-self.pose.x,2)+pow(self.reference.linear.y-self.pose.y,2))>tolerance) or (abs(self.reference.linear.z-self.deepness)>tolerance) or (abs(self.reference.angular.z-self.yaw)>tolerance_angular):#stablish control exit
            self.rate.sleep()
        return True
    def horizontal_plane_state(self, data):
        self.reference.linear.x=data.x#update reference value
        self.reference.linear.y=data.y
        self.control_horizontal=True#activate control
        while sqrt(pow(self.reference.linear.x-self.pose.x,2)+pow(self.reference.linear.y-self.pose.y,2))>tolerance:#stablish control exit
            self.rate.sleep()
        return True
    def heading_state(self, data):
        self.reference.angular.z=data.yaw#update reference value
        self.control_heading=True#activate control
        while abs(self.reference.angular.z-self.yaw)>tolerance_angular:#stablish control exit
            self.rate.sleep()
        return True




if __name__ == '__main__':
    try:
        x = controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('publisher::Exception')
    print('Leaving controller')



