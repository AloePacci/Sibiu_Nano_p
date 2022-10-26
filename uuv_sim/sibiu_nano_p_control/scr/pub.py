#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Point, Wrench
from sensor_msgs.msg import FluidPressure ,MagneticField, Imu, NavSatFix
from nav_msgs.msg import Odometry
import tf.transformations
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from math import pi

ns='/sibiu_nano_p' #namespace

rad_to_degree=180.0/pi
degree_to_rad=pi/180


class graficas:
    def __init__(self):
        
        rospy.init_node('grabadora', anonymous=True)#we init the node
        self.rate = rospy.Rate(10)
        while rospy.get_time()<1:#we wait until time starts
            self.rate.sleep() 
        self.time=rospy.get_time() #save time to check variation

        #we subscribe to the sensors
        self.pressure_sub=rospy.Subscriber(ns+'/pressure', FluidPressure, self.update_pressure)
        
        self.IMU_sub=rospy.Subscriber(ns+'/imu', Imu, self.update_imu)
        self.pose_sub=rospy.Subscriber(ns+'/pose_gt', Odometry, self.update_pose)
        self.reference_sub=rospy.Subscriber('controller/reference', Twist, self.update_reference)
        
        #we subscribe to the sensors we init the lists
        self.reference_t=[self.tiempo()]

        self.lista_p=[[], [], [], [], [], [], [], []]
        self.lista_p_t=[[], [], [], [], [], [], [], []]
        self.lista_a=[]
        self.lista_a_t=[]
        self.lista_angles=[[],[],[]]
        self.lista_angles_t=[]
        self.lista_h=[[],[]]
        self.lista_h_t=[]

        for ident in range(8):#we subscribe to the inputs
            self.thruster=rospy.Subscriber(ns+'/thrusters/'+str(ident)+'/input', FloatStamped, self.read_input, ident)
        
        #aux value to initialize reference
        aux=Twist()
        self.rate.sleep()
        self.rate.sleep()
        aux.linear.x=self.lista_h[0][0]
        aux.linear.y=self.lista_h[1][0]
        aux.linear.z=self.lista_a[0]*degree_to_rad
        aux.angular.z=self.lista_angles[2][0]
        self.reference=[aux]
    
    def tiempo(self):# this function returns time since program started runing
        return rospy.get_time()-self.time

        
    def read_input(self, data, ident):
        self.lista_p[ident].append(data.data)
        self.lista_p_t[ident].append(self.tiempo())

    def update_pose(self, data):
        self.lista_h[0].append(data.pose.pose.position.x)
        self.lista_h[1].append(data.pose.pose.position.y)
        self.lista_h_t.append(self.tiempo())


    def update_pressure(self, data):
        self.lista_a.append((data.fluid_pressure-101.325)*10/103.25) #deepness formula
        self.lista_a_t.append(self.tiempo())

    def update_imu(self, data):
        orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (self.roll,self.pitch,self.yaw) = tf.transformations.euler_from_quaternion(orientation)
        self.lista_angles[0].append(self.roll*rad_to_degree)
        self.lista_angles[1].append(self.pitch*rad_to_degree)
        self.lista_angles[2].append(self.yaw*rad_to_degree)
        self.lista_angles_t.append(self.tiempo())
        
    def update_reference(self, data):
        self.reference.append(self.reference[len(self.reference)-1])#as we want a vertical line for reference changes, we store the value just before and after the change
        self.reference.append(data)
        self.reference_t.append(self.tiempo()-0.001)
        self.reference_t.append(self.tiempo())

    def imprime(self):
    ##########################################
    # plot the data
    ##########################################
        self.reference_t.append(self.lista_a_t[len(self.lista_a_t)-1])
        

    #thrusters input

        fig=plt.figure(1)
        ax = fig.add_subplot(4, 2, 1)
        ax.plot(self.lista_p_t[0], self.lista_p[0],label='0')
        ax.grid(True)
        ax.set(xlabel='time', ylabel='input 1 (W)')
        ax = fig.add_subplot(4, 2, 2)
        ax.plot(self.lista_p_t[1], self.lista_p[1],label='1')
        ax.grid(True)
        ax.set(xlabel='time', ylabel='input 2 (W)')
        ax = fig.add_subplot(4, 2, 3)
        ax.plot(self.lista_p_t[2], self.lista_p[2],label='2')
        ax.grid(True)
        ax.set(xlabel='time', ylabel='input 3 (W)')
        ax = fig.add_subplot(4, 2, 4)
        ax.plot(self.lista_p_t[3], self.lista_p[3],label='3')
        ax.grid(True)
        ax.set(xlabel='time', ylabel='input 4 (W)')
        ax = fig.add_subplot(4, 2, 5)
        ax.plot(self.lista_p_t[4], self.lista_p[4],label='4')
        ax.grid(True)
        ax.set(xlabel='time', ylabel='input 5 (W)')
        ax = fig.add_subplot(4, 2, 6)
        ax.plot(self.lista_p_t[5], self.lista_p[5],label='5')
        ax.grid(True)
        ax.set(xlabel='time', ylabel='input 6 (W)')
        ax = fig.add_subplot(4, 2, 7)
        ax.plot(self.lista_p_t[6], self.lista_p[6],label='6')
        ax.grid(True)
        ax.set(xlabel='time', ylabel='input 7 (W)')
        ax = fig.add_subplot(4, 2, 8)
        ax.plot(self.lista_p_t[7], self.lista_p[7],label='7')
        ax.set(xlabel='time', ylabel='input 8 (W)')
        ax.grid(True)

        ##########################################
        #deepness

        fig2=plt.figure(2)
        ax2= fig2.add_subplot(1, 1, 1)
        ref_list=[]
        for i in self.reference:
            ref_list.append(i.linear.z)
        ref_list.append(ref_list[len(ref_list)-1])

        ax2.plot(self.lista_a_t, self.lista_a, color='tab:blue', label='altura')
        ax2.set(xlabel='time', ylabel='altura (m)')
        ax2.grid(True)
        ax2.legend()

        ##########################################
        #yaw

        ref_list=[]
        for i in self.reference:
            ref_list.append(i.angular.z*rad_to_degree)
        ref_list.append(ref_list[len(ref_list)-1])
 
        fig3=plt.figure(3)
        ax3 =fig3.add_subplot(1, 1, 1)
        ax3.plot(self.lista_angles_t, self.lista_angles[0], color='tab:blue', label='roll')
        ax3.plot(self.lista_angles_t, self.lista_angles[1], color='tab:orange',label='pitch')
        ax3.plot(self.lista_angles_t, self.lista_angles[2], color='tab:green',label='yaw')
        ax3.set(xlabel='time')
        ax3.grid(True)
        ax3.legend()



        ##########################################
        #horizontal position

        ref_list=[[],[]]
        for i in self.reference:
            ref_list[0].append(i.linear.x)
            ref_list[1].append(i.linear.y)
        fig4=plt.figure(4)
        ax4 = fig4.add_subplot(1, 1, 1)
        ax4.plot(self.lista_h[0], self.lista_h[1], color='tab:blue', label='position')
        ax4.set(xlabel='x', ylabel='y', title='position')
        ax4.grid(True)



        # set the limits
        #ax.set_xlim([0, 1])
        #ax.set_ylim([0, 1])
        #plt.plot(range(len(lista)),lista)
        plt.show()
        

if __name__ == '__main__':
    try:
        x = graficas()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('publisher::Exception')
    print('Leaving controller')
    x.imprime()



