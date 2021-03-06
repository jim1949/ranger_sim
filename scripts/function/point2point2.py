#实现点对点转向，其实速度为0，结束速度为0
from math import pi
import math


def point2point(point1,point2,orientation1,v,w,destflag,startflag,nearest_reading,corner_num):
    orientation2=math.atan2(point2.y-point1.y,point2.x-point1.x)
    angle=orientation2-orientation1
    flag_ped=nearest_reading<15
    flag_pavement=point1.y*point1.x/abs(point1.x)<8.0 and point1.y*point1.x/abs(point1.x)>0.0
    flag_line=(corner_num==0)or(corner_num==2)
    if (orientation2+pi)*180/pi<0.1:
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
                if orientation2-orientation1>pi/2:
                    w=2
                elif orientation2-orientation1<-pi/2:
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
                #add velocity controller here.
                v=v_controller(v,flag_ped,flag_pavement,flag_line,distance)
        else:
            w=(orientation2-orientation1)
            if abs(orientation2-orientation1)>1:
                w=0.05*(orientation2-orientation1)/abs(orientation2-orientation1)
            v=v_controller(v,flag_ped,flag_pavement,flag_line,distance)
            # v=8.0
    print("angle")
    print(angle)
    print("distance")
    print(distance)


    return v,w,destflag,startflag

    
def v_controller(v,flag_ped,flag_pavement,flag_line,distance):
    dt=1/60
    a=2 
    if flag_pavement==False:
        if distance<8.0 and flag_line:
            v=deaccelerator(v,a,dt)
            print(11111111)
        else:
            v=acceleretor(v,a,dt)
            print(22222222)
    else:
        if flag_ped==False:
            v=acceleretor(v,a,dt)
            print(33333333)
        else:
            if flag_line:
                v=deaccelerator(v,a,dt)
                print(444444444)

    return v

def acceleretor(v,a,dt):
    if v<8:
        v= v+a*dt
    else:
        v=8
    return v

def deaccelerator(v,a,dt):
    a=4
    if v>0:
        v= v-a*dt
    else:
        v=0
    return v



