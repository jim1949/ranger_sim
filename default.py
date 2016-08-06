#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <ranger_sim> environment

Feel free to edit this template as you like!
"""

from morse.builder import *
from morse.sensors import *
from ranger_sim.builder.robots import Mycar
from ranger_sim.builder.robots import Normalcar
from ranger_sim.builder.robots import Pedestrian
from ranger_sim.builder.actuators import Teleport2
import math

# Add the MORSE mascott, MORSY.
# Out-the-box available robots are listed here:
# http://www.openrobots.org/morse/doc/stable/components_library.html
#
# 'morse add robot <name> ranger_sim2' can help you to build custom robots.
#robot of car
robot = Mycar()

# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/stable/user/builder_overview.html
robot.translate(-1.5, -15.0, 1.0)
robot.rotate(0.0, 0.0, 1.57)

# robot2 = Normalcar()

# robot2.translate(1.5,-8.0, 1.0)
# robot2.rotate(0.0, 0.0, 1.57+3.14)



#pedestrains

people = Pedestrian()
people.translate(5.0, 0.0, 2.0)
people.rotate(0.0, 0.0, 0.0)

# people2=Pedestrian()
# people2.translate(-6.0, 10.0, 2.0)
# people2.rotate(0.0, 0.0, 3.14)


# motion2 = MotionVW()#collision
# motion2.translate(z=0.8)
# people.append(motion2)

# keyboard2 = Keyboard()
# people.append(keyboard2)
# keyboard2.properties(ControlType = 'Position')

# pose2 = Pose()
# people.append(pose2)

# people.add_default_interface('ros')




# # set 'fastmode' to True to switch to wireframe mode
# env = Environment('indoors-1/boxes', fastmode = False)
# set 'fastmode' to True to switch to wireframe mode
env = Environment('/Users/jj/morse/data/environments/indoors-1/crossing.blend', fastmode = False)
# env = Environment('land-1/rosace_1')
# env.set_camera_location([-18.0, -6.7, 10.8])
# env.set_camera_rotation([1.09, 0, -1.14])

env.set_camera_location([0, 0, 10.8])
env.set_camera_rotation([0, 0, -1.14])
