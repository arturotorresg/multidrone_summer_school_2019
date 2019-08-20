#!/usr/bin/env python
import rospkg
import rospy
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion

from uav_abstraction_layer.msg import State
from nav_msgs.msg import Odometry
import time
import signal
import sys
import numpy as np
import math as m
from numpy import linalg as l

unitz = np.array([0, 0, 1])
TO_DEG = 180/np.pi
drone_att_ = Quaternion()
gimbal_euler = np.array([0.0, 0.0, 0.0])

class GimbalControl:

    def __init__(self,drone_id):
        self.id = drone_id
        self.current_pose = PoseStamped()
        self.desired_pose = PoseStamped()
        self.gimbal_cmd_ = CommandLong()
        self.has_target_pose = 0;
        self.has_drone_pose = 0;

        # ROS Subscribers
        self.drone_pose_sub = rospy.Subscriber('ual/pose', PoseStamped, self.drone_pose_callback, queue_size=1)
        self.target_pose_sub = rospy.Subscriber('/drc_vehicle_xp900/odometry', Odometry, self.target_pose_callback, queue_size=1)

        # ROS Publishers

        # ROS Service clients
        self.cmd_long = rospy.ServiceProxy('mavros/cmd/command', CommandLong)

        # Start timer at 30Hz to control the drone
        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)


    def timer_callback(self,event):
        #---------------------------------------------------------------------------------------------------------#

        if (self.has_drone_pose == 1 and self.has_target_pose == 1):

            target_pos = np.array([self.target_pose.pose.pose.position.x, -self.target_pose.pose.pose.position.y, -self.target_pose.pose.pose.position.z])
            drone_pos = np.array([self.current_pose.pose.position.x, -self.current_pose.pose.position.y, -self.current_pose.pose.position.z])

            
            ###### YOUR CODE GOES HERE ######


            # q_I = np.array(np.array(target_pos - drone_pos))
            # r_x_3 = q_I/l.norm(q_I)
            # a = np.cross(r_x_3,-unitz)
            
            # r_x_2 = a/l.norm(a)
            # b = np.cross(r_x_2, r_x_3)

            # r_x_1 = b/l.norm(b)

            # R_x = np.transpose(np.matrix([r_x_1 ,r_x_2, r_x_3]))
            # self.rotationMatrixToEulerAngles_XYZ(R_x, gimbal_euler)

            #################################


            drone_att = self.current_pose.pose.orientation
            drone_yaw_ = m.atan2(2*(drone_att_.w*drone_att_.z+drone_att_.x*drone_att_.y),1-2*(drone_att_.y*drone_att_.y+drone_att_.z*drone_att_.z))

            self.gimbal_cmd_.param1 = gimbal_euler[1]*TO_DEG-90
            self.gimbal_cmd_.param2 = gimbal_euler[0]*TO_DEG
            self.gimbal_cmd_.param3 = gimbal_euler[2]*TO_DEG+drone_yaw_*TO_DEG

            try:
                self.cmd_long.wait_for_service()
                self.cmd_long.call(broadcast = False, confirmation = 0, command = 205, param1 = self.gimbal_cmd_.param1, param2 = self.gimbal_cmd_.param2, param3 = self.gimbal_cmd_.param3, param4 = 0, param5 = 0, param6 = 0, param7 = 2)

            except rospy.ServiceException,  e:
                print "\nService call failed: %s"%e 
        #---------------------------------------------------------------------------------------------------------#

    def drone_pose_callback(self,pose):
        self.current_pose = pose
        self.has_drone_pose = 1;

    def target_pose_callback(self,target_pose):
        self.target_pose = target_pose
        self.has_target_pose = 1;


    def rotationMatrixToEulerAngles_XYZ(self, R, euler):
        if (R[2,0] < 1):

            print m.atan2(R[2,1],R[2,2])
            print "oi\n"
            if (R[2,0] > -1):
                euler[1] = m.asin(-R[2,0]);
                euler[2] = m.atan2(R[1,0],R[0,0]);
                euler[0] = m.atan2(R[2,1],R[2,2]);
            else:
                euler[1] = np.pi/2;
                euler[2] = -m.atan2(-R[1,2],R[1,1]);
                euler[0] = 0;
        else: 
            euler[1] = -np.pi/2;
            euler[2] = m.atan2(-R[1,2],R[1,1]);
            euler[0] = 0;



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
