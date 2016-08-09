#using point2point rather than wayfollowing,change from car_control_original1.py version
#individual control script to control the car
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
path=np.array([[-1.5,20],[1.1,20],[1.1,-20],[-1.5,-20]])

# 2(1.5,20)->	3(1.5,-20)
# 	^			|
# 	|			v

# 1(-1.5,20)<-	4(-1.5)


def eulerfromquaterion(q0,q1,q2,q3):
	
	yaw=math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
	pitch=math.asin(2*(q0*q2-q3*q1))
	roll=math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
	print("steering angle")
	#experiment shows this is the yaw angle, and I don't know why.roll and yaw are different...(have changed the name already)
	print(yaw*180/pi)
	return roll,pitch,yaw

def callback1(msg):#position
	#=======================================#
    global car_x, car_y, car_theta
    global lineflag,lastlineflag,nearest_reading
    global v,w
    global destflag
    global corner_num
    

    position = msg.pose.position
    orientation = msg.pose.orientation

    roll,pitch,yaw = eulerfromquaterion(orientation.x,orientation.y,orientation.z,orientation.w)
    car_x = position.x
    car_y = position.y
    car_theta = yaw

    pt1=Point(position.x,position.y)
    pt2=Point(path[corner_num,0],path[corner_num,1])
#need to change this spee!
    v=8.0
    destflag=False
    v,w,destflag=point2point(pt1,pt2,yaw,v,w,destflag)
    if destflag==True:
        corner_num=corner_num+1
        if corner_num==4:
            corner_num=0
    print("corner_num")
    print(corner_num)
    # motion,lineflag,lastlineflag = waypoint_follower(lineflag, lastlineflag, position,yaw)

    if position.y*position.x/abs(position.x)<8.0 and position.y*position.x/abs(position.x)>0.0:
        v= 2.0
        #decrease the speed here
    if nearest_reading <= 8.0:
        v = 0.0


def callback2(msg):#velocity test
	linear=msg.twist.linear
	angular=msg.twist.angular

def callback3(msg):#scan and collision avoidance.
    ranges=msg.ranges
    angle_min=msg.angle_min
    angle_max=msg.angle_max
    intensities=msg.intensities
    scan_time=msg.scan_time
    global stopflag, nearest_reading
    stopflag=False
    for i in ranges[7:]:
        if i<5:
            stopflag=True

    # print(stopflag)
    nearest_reading = min(ranges[8:])

    # print("nearest reading: %2.2f" %nearest_reading)



cmd = rospy.Publisher("/robot/motion", Twist, queue_size = 10)
motion = Twist()
stopflag=False#flag if detect any pedestrians
rospy.init_node("car_controller")
rospy.Subscriber("/robot/pose", PoseStamped, callback1)
rospy.Subscriber("/robot/velocity", TwistStamped, callback2)
rospy.Subscriber("/robot/sick",LaserScan,callback3)
r=rospy.Rate(30)#30hz
while not rospy.is_shutdown():
    motion.linear.x=v
    motion.angular.z=w
    rospy.loginfo("Turn rate %f"%motion.angular.z)
    cmd.publish(motion)
    r.sleep()
	# rospy.Subscriber("/robot/velocity", TwistStamped, callback2)


# if __name__ == '__main__':
# 	listener()