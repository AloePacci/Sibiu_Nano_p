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
ns=rospy.get_param('/planner/namespace', '/sibiu_nano_p') 
debug=int(rospy.get_param('/planner/debug_level', 2)) 
interpolation_distance=float(rospy.get_param('/planner/interpolation_distance', 1.0)) 

#input parameter
reference=Twist()

#transformation coeficients
degree_to_rad=pi/180.0


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

def sentido(dist): #this function returns 1 or -1 depending on the given value
    if dist>0:
        return 1
    return -1

def dist(p1,p2):
    return sqrt(pow(p1-p2,2))

class planner:
    def __init__(self):
        rospy.init_node('planner_publisher', anonymous=True, log_level=debug_level(debug)) #init the node
        
        #subscribe to sensors
        self.pressure_sub=rospy.Subscriber(ns+'/pressure', FluidPressure, self.update_pressure)
        self.compass_sub=rospy.Subscriber(ns+'/magnetometer', MagneticField, self.update_magentometer)
        self.IMU_sub=rospy.Subscriber(ns+'/imu', Imu, self.update_imu)
        self.GPS_sub=rospy.Subscriber(ns+'/gps', NavSatFix, self.update_GPS)
        self.pose_sub=rospy.Subscriber(ns+'/pose_gt', Odometry, self.update_pose)

        #variables we will use
        self.rate = rospy.Rate(10) #10 Hz
        self.rate.sleep() #we wait until we have a first value for the sensors
        self.plan()

    def update_pose(self, data):
        self.pose=data.pose.pose.position
        orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        self.rot=tf.transformations.euler_from_quaternion(orientation) #[roll pitch yaw]


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

    def plan(self):
        #we interpolate heigth
        heigth_list=[self.deepness] #we set as first point the current heigth

        while abs(heigth_list[len(heigth_list)-1]-reference.linear.z)>interpolation_distance: #until we reach the goal point
            heigth_list.append(heigth_list[len(heigth_list)-1]+interpolation_distance*sentido(reference.linear.z-heigth_list[len(heigth_list)-1])) #we add the next heigth point within the interpolation distance
        heigth_list.append(reference.linear.z) #we add the goal point
        
        #we interpolate horizontal position
        list_x=[self.pose.x] #we set as first point the current position
        while dist(list_x[len(list_x)-1],reference.linear.x)>interpolation_distance: #until we reach the goal point
            list_x.append(list_x[len(list_x)-1]+interpolation_distance*sentido(reference.linear.x-list_x[len(list_x)-1])) #we add the next heigth point within the interpolation distance
        list_x.append(reference.linear.x) #we add the goal point

        list_y=[self.pose.y] #we set as first point the current position
        while dist(list_y[len(list_y)-1],reference.linear.y)>interpolation_distance: #until we reach the goal point
            list_y.append(list_y[len(list_y)-1]+interpolation_distance*sentido(reference.linear.y-list_y[len(list_y)-1])) #we add the next heigth point within the interpolation distance
        list_y.append(reference.linear.y) # we add the goal point


        #we make both lists have the same size
        while len(list_x)>len(list_y):
            list_y.append(list_y[len(list_y)-1])

        while len(list_y)>len(list_x):
            list_x.append(list_x[len(list_x)-1])
        
        horizontal_list=[]
        for i in range(len(list_x)):
            horizontal_list.append([list_x[i],list_y[i]])

        #we calculate how many times we call the service
        number=max([len(heigth_list),len(horizontal_list)])

        #both list must have the same size

        while len(heigth_list)<number:
            heigth_list.append(heigth_list[len(heigth_list)-1])

        while len(horizontal_list)<number:
            horizontal_list.append(horizontal_list[len(horizontal_list)-1])

        #we call the controller service
        rospy.wait_for_service('/controller/reference_position')
        for i in range(number-1):
            try:
                position_controller = rospy.ServiceProxy('/controller/reference_position', position)
                resp1 = position_controller(horizontal_list[i][0], horizontal_list[i][1], heigth_list[i],reference.angular.z)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        sys.exit(1)



if __name__ == '__main__':
    if len(sys.argv) == 5:        #asign the reference value
        reference.linear.x = float(sys.argv[1])
        reference.linear.y = float(sys.argv[2])
        reference.linear.z = float(sys.argv[3])
        reference.angular.z = float(sys.argv[4])*degree_to_rad
    else:
        print("incorrect call \nYou must use \"rosrun sibiu_nano_p_control planner x y z yaw\"")
        sys.exit(1)
    try:
        x = planner() #call the planner
        rospy.spin() #stay
    except rospy.ROSInterruptException:
        print('publisher::Exception')
    print('Leaving controller')

