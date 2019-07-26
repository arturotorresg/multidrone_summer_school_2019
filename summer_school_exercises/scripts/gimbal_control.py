#!/usr/bin/env python
import rospkg
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from uav_abstraction_layer.msg import State
from nav_msgs import Odometry
import time
import signal
import sys


class GimbalControl:

    def __init__(self,drone_id):
        self.id = drone_id
        self.current_pose = PoseStamped()
        self.desired_pose = PoseStamped()

        # ROS Subscribers
        self.drone_pose_sub = rospy.Subscriber('ual/pose', PoseStamped, self.drone_pose_callback, queue_size=1)
        self.target_pose_sub = rospy.Subscriber('/drc_vehicle_xp900/odometry', Odometry, self.target_pose_callback, queue_size=1)

        # ROS Publishers

        # ROS Service clients

        # Do some preliminary stuff
        

        # Start timer at 30Hz to control the drone
        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)


    def timer_callback(self,event):
        #---------------------------------------------------------------------------------------------------------#
        # Put your code here
        
        #---------------------------------------------------------------------------------------------------------#

    def drone_pose_callback(self,pose):
        self.current_pose = pose

    def target_pose_callback(self,target_pose):
        self.target_pose = target_pose


# Finish the execution directly when Ctrl+C is pressed (signal.SIGINT received), without escalating to SIGTERM.
def signal_handler(sig, frame):
    print('Ctrl+C pressed, signal.SIGINT received.')
    sys.exit(0)


if __name__ == "__main__":
    rospy.init_node('gimbal_control')

    drone_id = rospy.get_param('~drone_id', default=1)

    gimbal_control = GimbalControl(drone_id)

    # Associate signal SIGINT (Ctrl+C pressed) to handler (function "signal_handler")
    signal.signal(signal.SIGINT, signal_handler)

    rospy.spin()
