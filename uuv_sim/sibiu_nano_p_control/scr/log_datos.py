#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Point, Wrench
from sensor_msgs.msg import FluidPressure ,MagneticField, Imu, NavSatFix, BatteryState
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool
import tf.transformations
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from mavros_msgs.msg import State, RCOut

ns='/sibiu_nano_p' #namespace
#listas


class logger:
    def __init__(self):
        #nos suscribimos a los sensores
        
        rospy.init_node('grabadora', anonymous=True)
        self.altitude_service = rospy.Service('/save_data', CommandBool, self.save_data)

        self.lista_p_diff=[]
        self.lista_s_diff=[]
        self.lista_roll=[]
        self.lista_pitch=[]
        self.lista_yaw=[]
        self.lista_armed=[]
        self.lista_battery=[]
        self.lista_p_static=[]
        self.lista_h=[]
        self.lista_state=[]
        self.lista_channels=[]

        self.spressure_sub=rospy.Subscriber('/mavros/imu/static_pressure', FluidPressure, self.update_pressure_static)
        self.dpressure_sub=rospy.Subscriber('/mavros/imu/diff_pressure', FluidPressure, self.update_pressure_diff)
        self.IMU_sub=rospy.Subscriber('/mavros/imu/data', Imu, self.update_imu)
        self.state_sub=rospy.Subscriber('/mavros/state', State, self.update_state)
        self.rc_sub=rospy.Subscriber('/mavros/rc/out', RCOut, self.update_signal)
        self.battery_sub=rospy.Subscriber('/mavros/battery', BatteryState, self.update_battery)


    def update_state(self, data):
        self.lista_state.append(data)


    def update_pose(self, data):
        self.lista_h[0].append(data.pose.pose.position.x)

    def update_signal(self, data):
        self.lista_channels.append(data)

    def update_battery(self, data):
        self.lista_battery.append(data)

    def update_pressure_static(self, data):
        #self.lista_a.append((data.fluid_pressure-101.325)*10/103.25) #profundidad respecto al nivel del mar en m
        self.lista_p_static.append((data.fluid_pressure-101.325)*10/103.25) #profundidad respecto al nivel del mar en m

    def update_pressure_diff(self, data):
        #self.lista_a.append((data.fluid_pressure-101.325)*10/103.25) #profundidad respecto al nivel del mar en m
        self.lista_p_diff.append((data.fluid_pressure-101.325)*10/103.25) #profundidad respecto al nivel del mar en m
    def update_imu(self, data):
        orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (self.roll,self.pitch,self.yaw) = tf.transformations.euler_from_quaternion(orientation)
        self.lista_roll.append(self.roll)
        self.lista_pitch.append(self.pitch)
        self.lista_yaw.append(self.yaw)

    def save_data(self, data):
        file_p_diff = open('Save_data_pdif.txt', 'w')
        file_sdiff = open('Save_data_sdiff.txt', 'w')
        file_roll = open('Save_data_roll.txt', 'w')
        file_pitch = open('Save_data_pitch.txt', 'w')
        file_yaw = open('Save_data_yaw.txt', 'w')
        file_arm = open('Save_data_arm.txt', 'w')
        file_batter = open('Save_data_batter.txt', 'w')
        file_p_static = open('Save_data_p_static.txt', 'w')
        file_h = open('Save_data_h.txt', 'w')
        file_state = open('Save_data_state.txt', 'w')
        file_channels = open('Save_data_channels.txt', 'w')


        file_p_diff.write(str(self.lista_p_diff))
        file_sdiff.write(str(self.lista_s_diff))
        file_roll.write(str(self.lista_roll))
        file_pitch.write(str(self.lista_pitch))
        file_yaw.write(str(self.lista_yaw))
        file_arm.write(str(self.lista_armed))
        file_batter.write(str(self.lista_battery))
        file_p_static.write(str(self.lista_p_static))
        file_h.write(str(self.lista_h))
        file_state.write(str(self.lista_state))
        file_channels.write(str(self.lista_channels))
        return True
        

if __name__ == '__main__':
    try:
        x = graficas()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('publisher::Exception')
    print('Leaving controller')



