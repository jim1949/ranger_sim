#individual control script to control the car
import rospy
import message_filters#add message filters, this is one application package with ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import message_filters


def callback1(posemsg):
	#=======================================#
    position = posemsg.pose.position
    orientation = posemsg.pose.orientation


    # velocity: geometry_msgs/TwistStamped
    # linear = msg.twist.linear
    # angular = msg.twist.angular

    #
    #=======================================#
#     [geometry_msgs/PoseStamped]:
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# geometry_msgs/Pose pose
#   geometry_msgs/Point position
#     float64 x
#     float64 y
#     float64 z
#   geometry_msgs/Quaternion orientation
#     float64 x
#     float64 y
#     float64 z
#     float64 w
	#=======================================#

        # if position.x < 1:
        #     motion.linear.x = +0.5
        # if position.x > 2:
        #     motion.linear.x = -0.5
    motion.angular.z = +1
    # print("\norientation.x: ")
    # print(orientation.x)

    # print(",  orientation.y: ")
    # print(orientation.y)
    # print(",  orientation.z: ") 
    # print(orientation.z)  
    # print(",  orientation.w: ") 
    # print(orientation.w)

	print("\nposition.x: ")
	print(position.x)

	print("position.y: ")
	print(position.y)

	print("position.z: ")
	print(position.z)

	# motion.angular.z = +0.5
	cmd.publish(motion)

# def callback2(velocitymsg)
#     linear = velocitymsg.twist.linear
#     angular = velocitymsg.twist.angular
#     print("linear velocity:!!!!!!!!!!!!!!!!!!!!!!!!")
#     print(linear.x)


cmd = rospy.Publisher("/robot/motion", Twist, queue_size = 10)
motion = Twist()
t = 1
rospy.init_node("robot_car")
rospy.Subscriber("/robot/pose", PoseStamped, callback1)
	# rospy.Subscriber("/robot/velocity", TwistStamped, callback2)
rospy.spin() # this will block untill you hit Ctrl+C

# if __name__ == '__main__':
# 	listener()

