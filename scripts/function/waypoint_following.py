
from geometry_msgs.msg import Twist
from math import pi
import math
def waypoint_follower(lineflag, lastlineflag, position,yaw):
    motion = Twist()
    normal_velocity=8.0
    middle_shift=0.2
    if lineflag==0:
            if abs((yaw+(lastlineflag-1)*pi/2)*180/pi)<1 or (lastlineflag==4 and abs((yaw-pi/2)*180/pi)<1 ):
                motion.angular.z=0
                lineflag=lastlineflag+1
                if lineflag>4:
                    lineflag=1
                    lastlineflag=0
            else:
                motion.angular.z=-5*math.sin(yaw+(lastlineflag-1)*pi/2)

    if lineflag==1:
        if position.y<20:
            motion.linear.x=normal_velocity
            lineflag=1
        else:
            motion.linear.x=0.0
            lastlineflag=1
            lineflag=0
    if lineflag==2:
        if position.x<1.3-middle_shift:
            motion.linear.x=normal_velocity
        else:
            motion.linear.x=0.0
            lineflag=0
            lastlineflag=2
    if lineflag==3:
        if position.y>-20:
            motion.linear.x=normal_velocity
        else:
            motion.linear.x=0.0
            lineflag=0
            lastlineflag=3
    if lineflag==4:
        if position.x>-1.3-middle_shift:
            motion.linear.x=normal_velocity
        else:
            motion.linear.x=0.0
            lineflag=0
            lastlineflag=4
    print("motion.linear.x")
    print(motion.linear.x)
    print("motion.linear.y")
    print(motion.linear.y)
    print("lineflag")
    print(lineflag)
    print("lastlineflag")
    print(lastlineflag)

    return motion,lineflag,lastlineflag