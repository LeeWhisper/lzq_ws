import imp
from mimetypes import init
from traceback import format_exception

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes

class Nav():
    def __init__(self):
        rospy.init_node('navigation', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 5)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("waiting for move_base action server")
        self.move_base.wait_for_server(rospy.Duration(30))
        rospy.loginfo("connected to move_base server")

        loaction = Pose(Point(-3.33, 3.49, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = loaction
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()

        self.move_base.send_goal(self.goal)
    def shutdown(self):
        rospy.loginfo("stopping the car")
        self.move_base.cancel_goal()
        self.cmd_vel_pub.publish(Twist())

if __name__ == '__main__':
    sub = rospy.Publisher("123", BoundingBox, 1000)

    try:
        Nav()
        rospy.spin()

    except rospy.ROSInternalException:
        rospy.loginfo("navigation finished")
