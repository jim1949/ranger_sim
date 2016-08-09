#euler from quaterion function
# import math
# import rospy
# from tclass.point_rectangle import *


# # def eulerfromquaterion(q0,q1,q2,q3):
# # 	yaw=math.atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2))
# # 	pitch=math.asin(2*(q0*q2-q3*q1))
# # 	roll=math.atan2(2*(q0*q1+q2*q3),1-2*(q1^2+q2^2))
# # 	return yaw,pitch,roll

# # yaw,pitch,roll = eulerfromquaterion(orientation.x,orientation.y,orientation.z,orientation.w)
# pt1=Point()
# pt2=Point()
# # pt1.x=1
# # pt2.y=2
# # pt1.x=pt1.x+pt1.y

# pt1=Point(1,2)

# print(pt1.x)

# import numpy
# a=numpy.array([[-1.5,20],[1.1,20],[1.1,-20],[-1.5,-20]])
# print(a[0, 0])

# print(len(a))
# point = (1, 1)
# print(a[point])

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from math import pi

# from function.waypoint_following import waypoint_follower
from function.point2point import point2point
from tclass.point_rectangle import *
global nearest_reading, car_x, car_y, car_theta
global v,w
nearest_reading=1e7
v=0
w=0
global path
global corner_num
corner_num=0

pt1=Point(2,1)
pt2=Point(5,2)
orientation1=0.5
destflag=False

v,w,destflag=point2point(Point(2,1),Point(5,2),0.5,0,0,False)