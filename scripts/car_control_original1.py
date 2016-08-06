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
    global car_x, car_y, car_theta
    global lineflag,lastlineflag,nearest_reading

    position = msg.pose.position
    orientation = msg.pose.orientation
   
    # global information
    # information=information+0.1
    # print("1:")
    # print(information)
    # velocity: geometry_msgs/TwistStamped
    # linear = msg.twist.linear
    # angular = msg.twist.angular
    roll,pitch,yaw = eulerfromquaterion(orientation.x,orientation.y,orientation.z,orientation.w)
    car_x = position.x
    car_y = position.y
    car_theta = yaw

    #
    #=======================================#
#     [geometry_msgs/PoseStamped]:
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# geometry_msgs/Pose pose
#   geometry_msgs/Point position
#     float64 x
#     float64 y
#     float64 z
#   geometry_msgs/Quaternion orientation
#     float64 x
#     float64 y
#     float64 z
#     float64 w
	#=======================================#
	#basic motion control
    
    # if lineflag==0:
    #     if abs((yaw+(lastlineflag-1)*pi/2)*180/pi)<1 or (lastlineflag==4 and abs((yaw-pi/2)*180/pi)<1 ):
    #         motion.angular.z=0
    #         lineflag=lastlineflag+1
    #         if lineflag>4:
    #             lineflag=1
    #             lastlineflag=0
    #     else:
    #     	motion.angular.z=-5*math.sin(yaw+(lastlineflag-1)*pi/2)


    # if lineflag==1:
    #     if position.y<20:
    #         motion.linear.x=2.0
    #         lineflag=1
    #     else:
	   #      motion.linear.x=0.0
	   #      lastlineflag=1
	   #      lineflag=0
    # if lineflag==2:
    #     if position.x<1.5:
    # 	    motion.linear.x=2.0
    #     else:
    #         motion.linear.x=0.0
    #         lineflag=0
    #         lastlineflag=2
    # if lineflag==3:
    # 	if position.y>-20:
    # 	    motion.linear.x=2.0
    # 	else:
    #         motion.linear.x=0.0
    #         lineflag=0
    #         lastlineflag=3
    # if lineflag==4:
    # 	if position.x>-1.5:
    # 	    motion.linear.x=2.0
    # 	else:
    #         motion.linear.x=0.0
    #         lineflag=0
    #         lastlineflag=4
    # print("motion.linear.x")
    # print(motion.linear.x)
    # print("motion.linear.y")
    # print(motion.linear.y)
    # print("lineflag")
    # print(lineflag)
    # print("lastlineflag")
    # print(lastlineflag)
    # print("x!!")

    motion,lineflag,lastlineflag = waypoint_follower(lineflag, lastlineflag, position,yaw)

    if position.y*position.x/abs(position.x)<8.0 and position.y*position.x/abs(position.x)>0.0:
        motion.linear.x = 2.0
        #decrease the speed here
    if nearest_reading <= 8.0:

        motion.linear.x = 0.0;


    # if stopflag==True:
        # motion.linear.x=0
    print("final nearest_reading!!!!!!!!!!!!!!!!!!!!!!!!")
    print(nearest_reading)
    cmd.publish(motion)
	# motion.angular.z = +0.5


def callback2(msg):
	linear=msg.twist.linear
	angular=msg.twist.angular
	global lineflag
	# print("linear.x:")
	# print(linear.x)
	if lineflag == 1 or lineflag == 3:
		motion.angular.z =0.1*linear.x*linear.y/np.abs(linear.y)
	# else:
	# 	# print("angular.z")
	# 	# print(angular.z*180/pi)
		
	# 	if abs(angular.z+pi/2)<0.1:
	# 		motion.angular.z=0.0
	# 	else:
	# 	 	motion.angular.z=-0.1*(angular.z+pi/2)
def callback3(msg):
    ranges=msg.ranges
    angle_min=msg.angle_min
    angle_max=msg.angle_max
    intensities=msg.intensities
    scan_time=msg.scan_time
    global stopflag, nearest_reading

    print("ranges!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    stopflag=False
    for i in ranges[7:]:
        # print("I!!!!!!!!!!!!!!!!!!!!!!!")
        # print(i)
        if i<5:
            stopflag=True

    print(stopflag)
    nearest_reading = min(ranges[8:])

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
cmd = rospy.Publisher("/robot/motion", Twist, queue_size = 10)
# laser=rospy.
motion = Twist()
stopflag=False#flag if detect any pedestrians
lineflag=1#flag if it is on the straight line or steering, on which state of 4 edges of path.
lastlineflag=0
rospy.init_node("car_controller")

rospy.Subscriber("/robot/pose", PoseStamped, callback1)
# rospy.Subscriber("/robot/pose", PoseStamped, callback2)
rospy.Subscriber("/robot/velocity", TwistStamped, callback2)
rospy.Subscriber("/robot/sick",LaserScan,callback3)


	# rospy.Subscriber("/robot/velocity", TwistStamped, callback2)
rospy.spin() # this will block untill you hit Ctrl+C

# if __name__ == '__main__':
# 	listener()