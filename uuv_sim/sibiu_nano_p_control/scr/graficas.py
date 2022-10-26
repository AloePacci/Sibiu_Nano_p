#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Point, Wrench
from sensor_msgs.msg import FluidPressure ,MagneticField, Imu, NavSatFix
from nav_msgs.msg import Odometry
import tf.transformations
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped


ns='/sibiu_nano_p' #namespace
#listas


class graficas:
    def __init__(self):
        #nos suscribimos a los sensores
        
        rospy.init_node('grabadora', anonymous=True)

        self.pressure_sub=rospy.Subscriber(ns+'/pressure', FluidPressure, self.update_pressure)
        #self.compass_sub=rospy.Subscriber(ns+'/magnetometer', MagneticField, self.update_magentometer)
        self.IMU_sub=rospy.Subscriber(ns+'/imu', Imu, self.update_imu)
        self.pose_sub=rospy.Subscriber(ns+'/pose_gt', Odometry, self.update_pose)
        self.lista_p=[[], [], [], [], [], [], [], []]
        self.lista_a=[]
        self.lista_angles=[[],[],[]]
        self.lista_h=[[],[]]

        for ident in range(8):
            self.thruster=rospy.Subscriber(ns+'/thrusters/'+str(ident)+'/input', FloatStamped, self.read_input, ident)
    
    
    def read_input(self, data, ident):
        self.lista_p[ident].append(data.data)

    def update_pose(self, data):
        self.lista_h[0].append(data.pose.pose.position.x)
        self.lista_h[1].append(data.pose.pose.position.y)


    def update_pressure(self, data):
        self.lista_a.append((data.fluid_pressure-101.325)*10/103.25) #profundidad respecto al nivel del mar en m

    def update_imu(self, data):
        orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (self.roll,self.pitch,self.yaw) = tf.transformations.euler_from_quaternion(orientation)
        self.lista_angles[0].append(self.roll)
        self.lista_angles[1].append(self.pitch)
        self.lista_angles[2].append(self.yaw)

    def imprime(self):
    ##########################################
    # plot the data
    ##########################################

    #imprimir uso propulsores

        fig=plt.figure(1)
        ax = fig.add_subplot(8, 1, 1)
        ax.plot(range(len(self.lista_p[0])), self.lista_p[0], color='tab:blue', label='0')
        ax = fig.add_subplot(8, 1, 2)
        ax.plot(range(len(self.lista_p[1])), self.lista_p[1], color='tab:orange',label='1')
        ax = fig.add_subplot(8, 1, 3)
        ax.plot(range(len(self.lista_p[2])), self.lista_p[2], color='tab:green',label='2')
        ax = fig.add_subplot(8, 1, 4)
        ax.plot(range(len(self.lista_p[3])), self.lista_p[3], color='tab:red',label='3')
        ax = fig.add_subplot(8, 1, 5)
        ax.plot(range(len(self.lista_p[4])), self.lista_p[4], color='tab:purple',label='4')
        ax = fig.add_subplot(8, 1, 6)
        ax.plot(range(len(self.lista_p[5])), self.lista_p[5], color='tab:brown',label='5')
        ax = fig.add_subplot(8, 1, 7)
        ax.plot(range(len(self.lista_p[6])), self.lista_p[6], color='tab:pink',label='6')
        ax = fig.add_subplot(8, 1, 8)
        ax.plot(range(len(self.lista_p[7])), self.lista_p[7], color='tab:gray',label='7')
        ax.set(xlabel='time')
        ax.legend()

        ##########################################
        #imprimir altura

        fig2=plt.figure(2)
        ax2= fig2.add_subplot(1, 1, 1)
        ax2.plot(range(len(self.lista_a)), self.lista_a, color='tab:blue', label='altura')
        ax2.set(xlabel='time', ylabel='altura (m)')

        ##########################################
        #imprimir angulos
 
        fig3=plt.figure(3)
        ax3 =fig3.add_subplot(1, 1, 1)
        ax3.plot(range(len(self.lista_angles[0])), self.lista_angles[0], color='tab:blue', label='roll')
        ax3.plot(range(len(self.lista_angles[1])), self.lista_angles[1], color='tab:orange',label='pitch')
        ax3.plot(range(len(self.lista_angles[2])), self.lista_angles[2], color='tab:green',label='yaw')
        ax3.set(xlabel='time')
        ax3.legend()



        ##########################################
        #imprimir posicion horizontal

        fig4=plt.figure(4)
        ax4 = fig4.add_subplot(1, 1, 1)
        ax4.plot(self.lista_h[0], self.lista_h[1], color='tab:blue', label='position')
        ax4.set(xlabel='x', ylabel='y', title='position')



        # set the limits
        #ax.set_xlim([0, 1])
        #ax.set_ylim([0, 1])
        #plt.plot(range(len(lista)),lista)
        plt.grid(True)
        plt.show()
        

if __name__ == '__main__':
    try:
        x = graficas()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('publisher::Exception')
    print('Leaving controller')
    x.imprime



