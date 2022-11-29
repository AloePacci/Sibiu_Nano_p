#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Wrench
from sensor_msgs.msg import FluidPressure ,MagneticField, Imu, NavSatFix
from nav_msgs.msg import Odometry
import tf.transformations
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from mavros_msgs.msg import OverrideRCIn


ns=rospy.get_param('/planner/namespace', '/sibiu_nano_p') 
#listas


class translator:
    def __init__(self):
        #nos suscribimos a los sensores
        rospy.init_node('traductor', anonymous=True)
        
        for ident in range(8):
            self.thruster=rospy.Subscriber(ns+'/thrusters/'+str(ident)+'/input', FloatStamped, self.translate_input, ident)
        self.publisher=rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        self.override=OverrideRCIn()
        self.override.channels=[1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1100, 1400, 0, 0, 0, 0, 0, 0] #default values
        print(self.override.channels)
            
    
    
    def translate_input(self, data, ident):
        self.channel[indent]=1500.0+data.data
        self.publisher.publish(self.override)


if __name__ == '__main__':
    try:
        x = graficas()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('publisher::Exception')
    print('Leaving controller')



