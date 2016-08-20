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
# from function.point2point2 import point2point
# from tclass.point_rectangle import *

# class myClass():
#     def __init__(self):
#         self.mydata = []
#         self.myvalue = 1.4 
#     def print_values(self):
#         for item in self.mydata:
#             print item
#     def multi(self, x):
#         new_value = self.myvalue * x
#         self.myvalue  = new_value
#         return new_value


# A = myClass()
# B = myClass()

# A.multi(10)
# print(A.myvalue)
# print(B.myvalue)

class Point:
    
    """A point identified by (x,y) coordinates.
    
    supports: +, -, *, /, str, repr
    
    length  -- calculate length of vector to point from origin
    distance_to  -- calculate distance between two points
    as_tuple  -- construct tuple (x,y)
    clone  -- construct a duplicate
    integerize  -- convert x & y to integers
    floatize  -- convert x & y to floats
    move_to  -- reset x & y
    slide  -- move (in place) +dx, +dy, as spec'd by point
    slide_xy  -- move (in place) +dx, +dy
    rotate  -- rotate around the origin
    rotate_about  -- rotate around another point
    """
    
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y
    
    def __add__(self, p):
        """Point(x1+x2, y1+y2)"""
        return Point(self.x+p.x, self.y+p.y)
    
    def __sub__(self, p):
        """Point(x1-x2, y1-y2)"""
        return Point(self.x-p.x, self.y-p.y)
    
    def __mul__( self, scalar ):
        """Point(x1*x2, y1*y2)"""
        return Point(self.x*scalar, self.y*scalar)
    
    def __div__(self, scalar):
        """Point(x1/x2, y1/y2)"""
        return Point(self.x/scalar, self.y/scalar)
    
    def __str__(self):
        return "(%s, %s)" % (self.x, self.y)
    
    def __repr__(self):
        return "%s(%r, %r)" % (self.__class__.__name__, self.x, self.y)
    
    def length(self):
        return math.sqrt(self.x**2 + self.y**2)
    
    def distance_to(self, p):
        """Calculate the distance between two points."""
        return (self - p).length()
    
    def as_tuple(self):
        """(x, y)"""
        return (self.x, self.y)
    
    def clone(self):
        """Return a full copy of this point."""
        return Point(self.x, self.y)
    
    def integerize(self):
        """Convert co-ordinate values to integers."""
        self.x = int(self.x)
        self.y = int(self.y)
    
    def floatize(self):
        """Convert co-ordinate values to floats."""
        self.x = float(self.x)
        self.y = float(self.y)
    
    def move_to(self, x, y):
        """Reset x & y coordinates."""
        self.x = x
        self.y = y
    
    def slide(self, p):
        '''Move to new (x+dx,y+dy).
        
        Can anyone think up a better name for this function?
        slide? shift? delta? move_by?
        '''
        self.x = self.x + p.x
        self.y = self.y + p.y
    
    def slide_xy(self, dx, dy):
        '''Move to new (x+dx,y+dy).
        
        Can anyone think up a better name for this function?
        slide? shift? delta? move_by?
        '''
        self.x = self.x + dx
        self.y = self.y + dy
    
    def rotate(self, rad):
        """Rotate counter-clockwise by rad radians.
        
        Positive y goes *up,* as in traditional mathematics.
        
        Interestingly, you can use this in y-down computer graphics, if
        you just remember that it turns clockwise, rather than
        counter-clockwise.
        
        The new position is returned as a new Point.
        """
        s, c = [f(rad) for f in (math.sin, math.cos)]
        x, y = (c*self.x - s*self.y, s*self.x + c*self.y)
        return Point(x,y)
    
    def rotate_about(self, p, theta):
        """Rotate counter-clockwise around a point, by theta degrees.
        
        Positive y goes *up,* as in traditional mathematics.
        
        The new position is returned as a new Point.
        """
        result = self.clone()
        result.slide(-p.x, -p.y)
        result.rotate(theta)
        result.slide(p.x, p.y)
        return result

class trajectory_planner():
    def __init__(self):

        self.readings=[]
        self.car_x=0
        self.car_y=0
        self.car_theta=0
        self.car_ctrlSpeed=0
        self.car_ctrlSteer=0

        self.startflag=True
        self.destflag=False
        self.stopflag=False
        self.nearest_reading=1e7
        self.w=0
        self.v=0 
        rospy.init_node("car_controller")
        x=raw_input("Y or N?")
        if x=="Y"or"y":
            cmd = rospy.Publisher("/robot/motion", Twist, queue_size = 10)
            
            self.subscribe1=rospy.Subscriber("/robot/pose", PoseStamped, callback1)
            self.subscribe2=rospy.Subscriber("/robot/velocity", TwistStamped, callback2)
            self.subscirbe3=rospy.Subscriber("/robot/sick",LaserScan,callback3)
        else:
            cmd = rospy.Publisher("/robot/motion", Twist, queue_size = 10)
            
            self.subscirbe1=rospy.Subscriber("/pose", PoseStamped, callback1)
            self.subscirbe2=rospy.Subscriber("/velocity", TwistStamped, callback2)
            self.subscirbe3=rospy.Subscriber("/scan",LaserScan,callback3)
            self.sick_readings = open("/Users/jj/catkin_ws/src/carcontrol_pkg/src/tutorial_package/data/test1/sick_data.dat", "a")
            self.pedestrains_readings = open("/Users/jj/catkin_ws/src/carcontrol_pkg/src/tutorial_package/data/test1/pedestrains.dat", "a")
            self.results_file_handle = open("/Users/jj/catkin_ws/src/carcontrol_pkg/src/tutorial_package/data/test1/cardata.dat","a")



    def point2point(self,point1,point2,orientation1,v,w,destflag,startflag,nearest_reading,corner_num):
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
            self.v=0.0
            self.w=0.0

        else:
            destflag=False
            if startflag==True:
                if abs(angle)*180/pi>1:
                    self.v=0
                    if orientation2-orientation1>pi/2:
                        self.w=2
                    elif orientation2-orientation1<-pi/2:
                        self.w=-2
                    else:
                        # w=5*math.sin(orientation2-orientation1)#set the maximum angle velocity later on
                        self.w=5*(orientation2-orientation1)
                        if abs(orientation2-orientation1)>1:
                            self.w=(orientation2-orientation1)/abs(orientation2-orientation1)


                else:
                    self.w=0
                    # v=8.0
                    self.startflag=False
                    #add velocity controller here.
                    self.v=self.v_controller(v,flag_ped,flag_pavement,flag_line,distance)
            else:
                w=(orientation2-orientation1)
                if abs(orientation2-orientation1)>1:
                    w=0.05*(orientation2-orientation1)/abs(orientation2-orientation1)
                v=self.v_controller(v,flag_ped,flag_pavement,flag_line,distance)
                # v=8.0
        print("angle")
        print(angle)
        print("distance")
        print(distance)
        # print("velocity")
        # print(v)


        return v,w,destflag,startflag

        
    def v_controller(self,v,flag_ped,flag_pavement,flag_line,distance):
        dt=1.0/60
        a=2 
        if flag_pavement==False:
            if distance<8.0 and flag_line:
                v=deaccelerator(v,a,dt)
                print(11111111)
            else:
                print("velocity former")
                print(v)
                print("angular acc")
                print(a)
                v=acceleretor(v,a,dt)
                print(22222222)
                print("velocity")
                print(v)
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

    pt1=Point(car_x,car_y)
    pt2=Point(path[corner_num,0],path[corner_num,1])
    #need to change this spee!
    # v=8.0
    v,w,destflag,startflag=point2point(pt1,pt2,yaw,v,w,destflag,startflag,nearest_reading,corner_num)
    
    rospy.loginfo("destflag %f"%destflag)
    rospy.loginfo("startflag %f"%startflag)
    if destflag==True:
        corner_num=corner_num+1
        if corner_num==4:
            corner_num=0
    # print("corner_num")
    # print(corner_num)
    # motion,lineflag,lastlineflag = waypoint_follower(lineflag, lastlineflag, position,yaw)
    

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
    global sick_readings,reading
    ranges=msg.ranges
    angle_min=msg.angle_min
    angle_max=msg.angle_max
    intensities=msg.intensities
    scan_time=msg.scan_time
    global stopflag, nearest_reading
    stopflag=False
    readings=ranges
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

def start():
    global nearest_reading,readings, car_x, car_y, car_theta,car_ctrlSpeed, car_ctrlSteer
    global v,w
    global startflag,destflag
    global path
    global corner_num
    global sick_readings
    # readings=[]
    # car_x=0
    # car_y=0
    # car_theta=0
    # car_ctrlSpeed=0
    # car_ctrlSteer=0

    # startflag=True
    # destflag=False
    # nearest_reading=1e7
    # w=0
    # v=0
    tp=trajectory_planner()

    corner_num=0
    path=np.array([[-1.5,40],[1.1,40],[1.1,-40],[-1.5,-40]])
    # 2(1.5,20)->    3(1.5,-20)
    #     ^    |
    #     |    v

    # 1(-1.5,20)<-    4(-1.5)

    # stopflag=False#flag if detect any pedestrains
    motion=Twist()
    # print("got")
    # x=input("xxxx")
    r=rospy.Rate(60)#30hz

    while not rospy.is_shutdown():
        # results_file_handle = open("Published_results.dat","a")
        
        motion.linear.x=v
        motion.angular.z=w
        rospy.loginfo("Turn rate %f"%motion.angular.z)
        rospy.loginfo("Velocity %f"%motion.linear.x)

        results_file_handle.write("%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f \n" %(rospy.get_time(), car_x, car_y, car_theta, car_ctrlSpeed, car_ctrlSteer))
        pedestrains_readings.write("%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f \n" %(rospy.get_time(), car_x, car_y, car_theta, car_ctrlSpeed, car_ctrlSteer))
        if not readings:
            for value in readings:
                pedestrains_readings.write("%2.4f, " %value)
        pedestrains_readings.write("\n")
        cmd.publish(motion)
        r.sleep()
        # rospy.Subscriber("/robot/velocity", TwistStamped, callback2)
    rospy.spin()

if __name__ == '__main__':
    start()