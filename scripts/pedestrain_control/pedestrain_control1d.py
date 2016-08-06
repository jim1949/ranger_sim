import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as numpy
import math
from math import pi

global p_v,p_w
p_v=0
p_w=0

def eulerfromquaterion(q0,q1,q2,q3):
    
    yaw=math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
    pitch=math.asin(2*(q0*q2-q3*q1))
    roll=math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
 #    print("steering angle")
	# #experiment shows this is the yaw angle, and I don't know why.roll and yaw are different...(have changed the name already)
 #    print(yaw*180/pi)
    return roll,pitch,yaw

def callback(msg):
    global p_v,p_w
    position=msg.pose.position
    orientation=msg.pose.orientation
    roll,pitch,yaw = eulerfromquaterion(orientation.x,orientation.y,orientation.z,orientation.w)




rospy.init_node("pedestrain_controller")
pub=rospy.Publisher("/people/motion",Twist, queue_size=10)
rospy.Subscriber("/people/pose", PoseStamped, callback)
r=rospy.Rate(1)#1hz
motion=Twist()
while not rospy.is_shutdown():
    # motion=Twist()
    motion.linear.x=-1
    motion.angular.z=motion.angular.z+0.2
    rospy.loginfo("Turn rate %f"%motion.angular.z)
    pub.publish(motion)
    r.sleep()
    # rate.sleep()
    
# if __name__=='__main__':
#     try:
#         pedestrain_control1()
#     except rospy.ROSInterruptException:
#         pass