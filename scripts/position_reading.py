#individual control script to control the car
import rospy
import message_filters#add message filters, this is one application package with ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
import message_filters
import numpy as np
import math
from math import pi

from waypoint_following import waypoint_follower
global nearest_reading, car_x, car_y, car_theta
global v,w
nearest_reading=1e7
global stopflag,lineflag,lastlineflag
stopflag=False#flag if detect any pedestrians
lineflag=1#flag if it is on the straight line or steering, on which state of 4 edges of path.
lastlineflag=0

def eulerfromquaterion(q0,q1,q2,q3):
    
    yaw=math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
    pitch=math.asin(2*(q0*q2-q3*q1))
    roll=math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
    print("steering angle")
    #experiment shows this is the yaw angle, and I don't know why.roll and yaw are different...(have changed the name already)
    print(yaw*180/pi)
    return roll,pitch,yaw

def callback1(msg):
    #=======================================#
    global car_x, car_y, car_theta,v,w
    global lineflag,lastlineflag,nearest_reading

    position = msg.pose.position
    orientation = msg.pose.orientation

    roll,pitch,yaw = eulerfromquaterion(orientation.x,orientation.y,orientation.z,orientation.w)
    car_x = position.x
    car_y = position.y
    car_theta = yaw

    motion,lineflag,lastlineflag = waypoint_follower(lineflag, lastlineflag, position,yaw)

    if nearest_reading <= 7.0:

        motion.linear.x = 0.0;

    # if stopflag==True:
        # motion.linear.x=0
    print("velocity!!!!!!!!!!!!!!!!!!!!!!!!")
    print(motion.linear.x)
    return motion
    v=motion.linear.x
    w=motion.linear.z
    # motion.angular.z = +0.5

def listener():
    rospy.init_node('robot_position', anonymous=True)
    rospy.Subscriber("/robot/pose", PoseStamped, callback1)
    rospy.spin()

if __name__ == '__main__':
    listener()