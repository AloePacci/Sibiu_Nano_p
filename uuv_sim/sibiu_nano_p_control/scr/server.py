#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from sibiu_nano_p_control.cfg import TutorialsConfig
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

def callback(config, level):
    #rospy.loginfo("""Reconfigure Request: {Thruster_1}, {Thruster_2}, {Thruster_3}, {Thruster_4}, {Thruster_5}, {Thruster_6}, {Thruster_7}, {Thruster_8}""".format(**config))
    message=FloatStamped()
    data=[float("{Thruster_1}".format(**config)), float("{Thruster_2}".format(**config)), float("{Thruster_3}".format(**config)), float("{Thruster_4}".format(**config)), float("{Thruster_5}".format(**config)), float("{Thruster_6}".format(**config)), float("{Thruster_7}".format(**config)), float("{Thruster_8}".format(**config))]
    print(data)

    for ident in range(8):#we subscribe to the inputs
        thruster=rospy.Publisher('/sibiu_nano_p/thrusters/'+str(ident)+'/input', FloatStamped, queue_size=10)
        message.data=data[ident]
        thruster.publish(message)
    return config


if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = False)
    srv = Server(TutorialsConfig, callback)
    rospy.spin()
