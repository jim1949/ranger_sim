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
# from function.point2point import point2point
from function.point2point1 import point2point
from tclass.point_rectangle import *
global nearest_reading, car_x, car_y, car_theta,car_ctrlSpeed, car_ctrlSteer
car_x=0
car_y=0
car_theta=0
car_ctrlSpeed=0
car_ctrlSteer=0
global v,w
global startflag,destflag
startflag=True
destflag=False
nearest_reading=1e7
w=0
v=0
global path
global corner_num
corner_num=0
path=np.array([[-1.5,20],[1.1,20],[1.1,-20],[-1.5,-20]])

global sick_readings
sick_readings = open("sick_data2.dat", "a")
# 2(1.5,20)->    3(1.5,-20)
#     ^    |
#     |    v

# 1(-1.5,20)<-    4(-1.5)


def eulerfromquaterion(q0,q1,q2,q3):
    
    yaw=math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
    pitch=math.asin(2*(q0*q2-q3*q1))
    roll=math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
    # print("steering angle")
    # #experiment shows this is the yaw angle, and I don't know why.roll and yaw are different...(have changed the name already)
    # print(yaw*180/pi)
    return roll,pitch,yaw

def callback1(msg):#position
    #=======================================#
    global car_x, car_y, car_theta
    global lineflag,lastlineflag,nearest_reading
    global v,w
    global destflag,startflag
    global corner_num

    fid = open('nearest_reading.dat', 'a')

    position = msg.pose.position
    orientation = msg.pose.orientation
    rospy.loginfo("positionx %f"%position.x)
    rospy.loginfo("positiony %f"%position.y)
    roll,pitch,yaw = eulerfromquaterion(orientation.x,orientation.y,orientation.z,orientation.w)
    car_x = position.x
    car_y = position.y
    car_theta = yaw

    pt1=Point(position.x,position.y)
    pt2=Point(path[corner_num,0],path[corner_num,1])
#need to change this spee!
    v=8.0
    
    v,w,destflag,startflag=point2point(pt1,pt2,yaw,v,w,destflag,startflag)
    rospy.loginfo("destflag %f"%destflag)
    rospy.loginfo("startflag %f"%startflag)
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
    print("nearest_reading")
    print(nearest_reading)
    fid.write("%s\n" %nearest_reading)

def callback2(msg):#velocity test
    global car_ctrlSpeed,car_ctrlSteer
    linear=msg.twist.linear
    angular=msg.twist.angular
    car_ctrlSpeed=math.sqrt(linear.x**2+linear.y**2+linear.z**2)
    car_ctrlSteer=angular.z



def callback3(msg):#scan and collision avoidance.
    global sick_readings
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
    mytime = rospy.get_time()
    sick_readings.write("%2.2f, " %mytime)
    for value in ranges:
        sick_readings.write("%2.4f, " %value)

    sick_readings.write("\n")
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
    # results_file_handle = open("Published_results.dat","a")
    results_file_handle = open("testdata","a")
    motion.linear.x=v
    motion.angular.z=w
    rospy.loginfo("Turn rate %f"%motion.angular.z)
    rospy.loginfo("Velocity %f"%motion.linear.x)

    # rospy.loginfo("time %f"%rospy.get_time())
    # rospy.loginfo("car_x %f"%car_x)
    # rospy.loginfo("car_y %f"%car_y)
    # rospy.loginfo("car_theta %f"%car_theta)
    # rospy.loginfo("car_ctrlSpeed %f"%car_ctrlSpeed)
    # rospy.loginfo("car_ctrlSteer %f"%car_ctrlSteer)

    results_file_handle.write("%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f \n" %(rospy.get_time(), car_x, car_y, car_theta, car_ctrlSpeed, car_ctrlSteer))
    cmd.publish(motion)
    r.sleep()
    # rospy.Subscriber("/robot/velocity", TwistStamped, callback2)


# if __name__ == '__main__':
#     listener()