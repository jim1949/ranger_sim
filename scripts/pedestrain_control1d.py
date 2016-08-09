import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tclass.point_rectangle import *
from function.point2point1 import point2point
import numpy as np
import math
from math import pi
import random


global v,w,dest
global startflag,destflag
startflag=True
destflag=False
v=0
w=0
dest=8

global path
global corner_num
corner_num=0
path=np.array([[-1.5,0],[1.1,0],[9,0],[-5,0]])
def eulerfromquaterion(q0,q1,q2,q3):
    
    yaw=math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
    pitch=math.asin(2*(q0*q2-q3*q1))
    roll=math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
 #    print("steering angle")
    # #experiment shows this is the yaw angle, and I don't know why.roll and yaw are different...(have changed the name already)
 #    print(yaw*180/pi)
    return roll,pitch,yaw

def callback(msg):
    global v,w
    global dest
    global destflag,startflag
    global corner_num

    position=msg.pose.position
    orientation=msg.pose.orientation
    roll,pitch,yaw = eulerfromquaterion(orientation.x,orientation.y,orientation.z,orientation.w)
    
    pt1=Point(position.x,position.y)
    pt2=Point(dest,0)
    # print(pt1.x)
    # print(pt1.y)
    # print(pt2.x)
    # print(pt2.y)
    # print(yaw)
 
    pt1=Point(position.x,position.y)
    pt2=Point(path[corner_num,0],path[corner_num,1])
#need to change this spee!
    v=3.0
    
    v,w,destflag,startflag=point2point(pt1,pt2,yaw,v,w,destflag,startflag)
    rospy.loginfo("destflag %f"%destflag)
    rospy.loginfo("startflag %f"%startflag)
    if destflag==True:
        corner_num=corner_num+1
        if corner_num==4:
            corner_num=0
    print("corner_num")
    print(corner_num)



rospy.init_node("pedestrain_controller")
pub=rospy.Publisher("/people/motion",Twist, queue_size=10)
rospy.Subscriber("/people/pose", PoseStamped, callback)
r=rospy.Rate(20)#1hz
motion=Twist()
while not rospy.is_shutdown():
    # motion=Twist()

    motion.linear.x=v
    motion.angular.z=w
    # if startflag==False and destflag==False and 
    rospy.loginfo("Turn rate %f"%motion.angular.z)
    rospy.loginfo("Velocity %f"%motion.linear.x)
    pub.publish(motion)
    r.sleep()
    # rate.sleep()
#last pedestrain version 8.9   
# if __name__=='__main__':
#     try:
#         pedestrain_control1()
#     except rospy.ROSInterruptException:
#         pass