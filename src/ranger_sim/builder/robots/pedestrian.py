from morse.builder import *

class Pedestrian(GroundRobot):
    """
    A template robot model for pedestrian, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

        # pedestrian.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'ranger_sim/robots/pedestrian.blend', name)
        self.properties(classpath = "ranger_sim.robots.pedestrian.Pedestrian")

        ###################################
        # Actuators
        ###################################


        # (v,w) motion controller
        # Check here the other available actuators:
        # http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
        self.motion = MotionVW()
        self.append(self.motion)

        # Optionally allow to move the robot with the keyboard
        if debug:
            keyboard = Keyboard()
            keyboard.properties(ControlType = 'Position')
            self.append(keyboard)

        ###################################
        # Sensors
        ###################################

        self.pose = Pose()
        self.append(self.pose)
#         motion2 = MotionVW()#collision
# motion2.translate(z=0.8)
# people.append(motion2)

# keyboard2 = Keyboard()
# people.append(keyboard2)
# keyboard2.properties(ControlType = 'Position')

# pose2 = Pose()
# people.append(pose2)

        self.pose.add_stream("ros", "morse.middleware.ros.pose.PoseStampedPublisher", topic = "/people/pose")
        self.pose.add_service("ros")
        
        self.motion.add_stream("ros","morse.middleware.ros.motion_vw.TwistReader", topic = "/people/motion")
        self.motion.add_service("ros")


