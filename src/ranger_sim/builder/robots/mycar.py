from morse.builder import *
from ranger_sim.builder.actuators import Teleport2

class Mycar(GroundRobot):
    """
    A template robot model for mycar, with a motion controller and a pose sensor.
    """
    
    def __init__(self, name = None, debug = False):#change the debug model to false because I don't want to use keyboard control the robot

        # mycar.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'ranger_sim/robots/mycar.blend', name)
        self.properties(classpath = "ranger_sim.robots.mycar.Mycar")

        ###################################
        # Actuators
        ###################################


        # (v,w) motion controller
        # Check here the other available actuators:
        # http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
        self.motion = MotionVW()
        self.append(self.motion)

        self.teleport2 = Teleport2()
        self.append(self.teleport2)
        self.teleport2.add_interface('ros')



        # Optionally allow to move the robot with the keybo ard
        if debug:
            keyboard = Keyboard()
            keyboard.properties(ControlType = 'Position')
            self.append(keyboard)

        ###################################
        # Sensors
        ###################################

        self.pose = Pose()
        self.append(self.pose)

        self.sick = Sick()
        self.append(self.sick)
        self.sick.translate(0.0,0.0,0.2)
        self.sick.properties(resolution = 5)
        self.sick.properties(scan_window = 90)
        self.sick.properties(laser_range = 5.0)
        self.sick.properties(Visible_arc=True)

        self.pose.add_stream("ros", "morse.middleware.ros.pose.PoseStampedPublisher", topic = "/robot/pose")
        self.pose.add_service("ros")
        
        self.motion.add_stream("ros","morse.middleware.ros.motion_vw.TwistReader", topic = "/robot/motion")
        self.motion.add_service("ros")

        # self.velocity.add_stream("ros","morse.middleware.ros.velocity.TwistStampedPublisher", topic = "/robot/velocity")
        # self.velocity.add_service("ros")

        self.teleport2.add_stream("ros","morse.middleware.ros.read_pose.PoseReader", topic = "/robot/teleport")
        self.teleport2.add_service("ros")
       




