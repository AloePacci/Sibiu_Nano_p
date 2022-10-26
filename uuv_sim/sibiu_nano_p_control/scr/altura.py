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
from graficas import graficas
from gazebo_msgs.msg import ApplyBodyWrench
rosservice call /gazebo/apply_body_wrench '{body_name: "sibiu_nano_p/thruster_5_joint" , wrench: { torque: { x: -0.01, y: 0 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }'




rosservice call /gazebo/apply_joint_effort "joint_name: 'sibiu_nano_p/thruster_5_joint'
effort: 2.0
start_time:
  secs: 0
  nsecs: 0
duration:
  secs: 1
  nsecs: 0" 
