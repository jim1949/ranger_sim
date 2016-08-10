#实现点对点转向，其实速度为0，结束速度为0
from math import pi
import math


def point2point(point1,point2,orientation1,v,w,destflag,startflag):
    orientation2=math.atan2(point2.y-point1.y,point2.x-point1.x)
    angle=orientation2-orientation1
    if (orientation2+pi)*180/pi<1:
        orientation2=pi


    distance=point1.distance_to(point2)
    if distance<0.1:
        destflag=True
        startflag=True
        v=0.0
        w=0.0

    else:
        destflag=False
        if startflag==True:
            if abs(angle)*180/pi>1:
                v=0
                if orientation2-orientation1>pi:
                    w=2
                elif orientation2-orientation1<-pi:
                    w=-2
                else:
                    # w=5*math.sin(orientation2-orientation1)#set the maximum angle velocity later on
                    w=5*(orientation2-orientation1)
                    if abs(orientation2-orientation1)>1:
                        w=(orientation2-orientation1)/abs(orientation2-orientation1)


            else:
                w=0
                # v=8.0
                startflag=False

        else:
            w=(orientation2-orientation1)
            if abs(orientation2-orientation1)>1:
                w=0.05*(orientation2-orientation1)/abs(orientation2-orientation1)

            # v=8.0
    print("angle")
    print(angle)
    print("distance")
    print(distance)



    return v,w,destflag,startflag


