#point2point
from math import pi
import math


def point2point(point1,point2,orientation1,v,w,destflag):
    max_w=2#maximum angular velocity
    # a=2#acceleration velocity
    # freq=10#publishing frequency
    # dt=1/freq

    #pt1 w<0,v=0-----w,v!=0------->pt2 w<0,v=0
    
    orientation2=math.atan2(point2.y-point1.y,point2.x-point1.x)
    print("orientation2!!!!!!!!!!!!!!!!!!!!!!")
    print(orientation2)
    print(orientation1)
    nearangle=abs((orientation2-orientation1)*180/pi)<1 or abs((orientation2-orientation1+2*pi)*180/pi)<1 or abs((orientation2-orientation1-2*pi)*180/pi)<1
    rangeangle=abs((orientation2-orientation1)*180/pi)<30 or abs((orientation2-orientation1+2*pi)*180/pi)<30 or abs((orientation2-orientation1-2*pi)*180/pi)<30
    if destflag==True:
        v=0
    else:
        v=3.0
    if nearangle:#straightway
        w=0
        print("got there!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        # destflag=False
        # v=v+a*dt
    #need to set the angular velocity

    else:#turn the angle
        # w=-2
        # if rangeangle:
        w=5*math.sin(orientation2-orientation1)

        # if abs(w)>2:
        #     w=-2*(orientation2-orientation1)/abs((orientation2-orientation1))

    # distance=math.sqrt((point1.x-point2.x)**2+(point1.y-point2.y)**2)
    distance=point1.distance_to(point2)
    if distance<0.1:
        destflag=True
        #need to change the control part here
        print("here I amm!!!!!!!!!!!!!!!!!!!!!!!!!>>>>>>>>>>>>")
        v=0
    print("point2.x")
    print(point2.x)
    print(point1.x)
    print("point2.y")
    print(point2.y)
    print(point1.y)
    print("distance")
    print(distance)

    return v,w,destflag

