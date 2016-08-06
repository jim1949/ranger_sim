#individual control script to control the car
import rospy
import message_filters#add message filters, this is one application package with ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import message_filters
import numpy as np
import math
from math import pi

from waypoint_following import waypoint_follower

global nearest_reading, car_x, car_y, car_theta
global v,w
nearest_reading=1e7


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


def callback2(msg):
    motion=Twist()
    linear=msg.twist.linear
    angular=msg.twist.angular
    global lineflag,v,w
    # print("linear.x:")
    # print(linear.x)
    if lineflag == 1 or lineflag == 3:
        w=0.1*linear.x*linear.y/np.abs(linear.y)
    # else:
    #     # print("angular.z")
    #     # print(angular.z*180/pi)
        
    #     if abs(angular.z+pi/2)<0.1:
    #         motion.angular.z=0.0
    #     else:
    #          motion.angular.z=-0.1*(angular.z+pi/2)
def callback3(msg):
    ranges=msg.ranges
    angle_min=msg.angle_min
    angle_max=msg.angle_max
    intensities=msg.intensities
    scan_time=msg.scan_time
    global stopflag, nearest_reading,v,w

    print("ranges!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    stopflag=False
    for i in ranges:
        # print("I!!!!!!!!!!!!!!!!!!!!!!!")
        # print(i)
        if i<5:
            stopflag=True

    print(stopflag)
    nearest_reading = min(ranges)

    print("nearest reading: %2.2f" %nearest_reading)


    #laserscanner
    #=======================================#
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# float32 angle_min
# float32 angle_max
# float32 angle_increment
# float32 time_increment
# float32 scan_time
# float32 range_min
# float32 range_max
# float32[] ranges
# float32[] intensities
    #=======================================#




#straightline or turning angle




def talker():
    global v,w
    cmd = rospy.Publisher("/robot/motion", Twist, queue_size = 10)
# laser=rospy.
    motion = Twist()
    global stopflag,lineflag,lastlineflag
    stopflag=False#flag if detect any pedestrians
    lineflag=1#flag if it is on the straight line or steering, on which state of 4 edges of path.
    lastlineflag=0
    v=0
    w=0
    # motion=Twist()
    rospy.Subscriber("/robot/pose", PoseStamped, callback1)
# rospy.Subscriber("/robot/pose", PoseStamped, callback2)
    rospy.Subscriber("/robot/velocity", TwistStamped, callback2)
    rospy.Subscriber("/robot/sick",LaserScan,callback3)
    rospy.init_node("robot_car")
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        global v,w
        rospy.loginfo("nearest reading: %2.2f" %nearest_reading)
        rospy.loginfo("velocity:%2.2f"%motion.linear.x)
        motion.linear.z=w
        motion.linear.x=v
        cmd.publish(motion)
    # rospy.Subscriber("/robot/velocity", TwistStamped, callback2)
        rate.sleep()
        # rospy.spin() # this will block untill you hit Ctrl+C

if __name__ == '__main__':
    talker()

