#! /usr/bin/env python3
"""
Test client for the <ranger_sim> simulation environment.

This simple program shows how to control a robot from Python.

For real applications, you may want to rely on a full middleware,
like ROS (www.ros.org).
"""

import sys
from geometry_msgs.msg import Pose
# from tf.transformations import quaternion_from_euler

try:
    from pymorse import Morse
except ImportError:
    print("you need first to install pymorse, the Python bindings for MORSE!")
    sys.exit(1)

print("Use WASD to control the robot")

with Morse() as simu:

  motion = simu.robot.motion
  pose = simu.robot.pose
  myPose=Pose()

 


  v = 0.0
  w = 0.0
  yaw=0.0
  pitch=0.0
  roll=0.0
  # q = quaternion_from_euler(yaw, pitch, roll, 'rzyx')

  while True:

      """
A triple of Euler angles can be applied/interpreted in 24 ways, which can
be specified using a 4 character string or encoded 4-tuple:

  *Axes 4-string*: e.g. 'sxyz' or 'ryxy'

  - first character : rotations are applied to 's'tatic or 'r'otating frame
  - remaining characters : successive rotation axis 'x', 'y', or 'z'
"""


      key = input("WASDR?")

      if key.lower() == "w":
          v += 0.1
      elif key.lower() == "s":
          v -= 0.1
      elif key.lower() == "a":
          w += 0.1
      elif key.lower() == "d":
          w -= 0.1
      elif key.lower() == "r":
          myPose.position.x=10.5
          myPose.position.y=44.25
          myPose.position.z=1.0
          # myPose.orientation.x = q[0]
          # myPose.orientation.y = q[1]
          # myPose.orientation.z = q[2]
          # myPose.orientation.w = q[3]
          tp_pub = rospy.Publisher("/robot/teleport", Pose, queue_size=1)
          tp_pub.publish(myPose)
          continue


      else:
          continue

      # here, we call 'get' on the pose sensor: this is a blocking
      # call. Check pymorse documentation for alternatives, including
      # asynchronous stream subscription.
      print("The robot is currently at: %s\n" % pose.get())

      motion.publish({"v": v, "w": w})
