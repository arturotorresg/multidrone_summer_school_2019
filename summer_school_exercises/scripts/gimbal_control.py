#!/usr/bin/env python
import rospkg
import rospy
from mavros_msgs.srv import CommandLong, ParamGet, ParamSet
from mavros_msgs.msg import ParamValue
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
        self.has_target_pose = False
        self.has_drone_pose = False

        # ROS Subscribers
        self.drone_pose_sub = rospy.Subscriber('ual/pose', PoseStamped, self.drone_pose_callback, queue_size=1)
        self.target_pose_sub = rospy.Subscriber('/drc_vehicle_xp900/odometry', Odometry, self.target_pose_callback, queue_size=1)

        # ROS Publishers

        # ROS Service clients
        self.cmd_long_service = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
        self.param_get_service = rospy.ServiceProxy('mavros/param/get', ParamGet)
        self.param_set_service = rospy.ServiceProxy('mavros/param/set', ParamSet)

        # Wait for ROS services to be ready
        self.cmd_long_service.wait_for_service()
        self.param_get_service.wait_for_service()
        self.param_set_service.wait_for_service()

        rospy.loginfo('Drone %d: setting up gimbal control'%self.id)

        # Prepare gimbal to work
        self.set_mount_mode_parameters()

        # Start timer at 30Hz to control the drone
        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)

        rospy.loginfo('Drone %d: gimbal control running'%self.id)
        print ''


    def timer_callback(self,event):
        if (self.has_drone_pose and self.has_target_pose):

            target_pos = np.array([self.target_pose.pose.pose.position.x, -self.target_pose.pose.pose.position.y, -self.target_pose.pose.pose.position.z])
            drone_pos = np.array([self.current_pose.pose.position.x, -self.current_pose.pose.position.y, -self.current_pose.pose.position.z])

            #-----------------------------------------------------------------#
            # Put your code here

            

            #-----------------------------------------------------------------#

            self.rotation_matrix_to_euler_angles_xyz(R_x, gimbal_euler)
            drone_att = self.current_pose.pose.orientation
            drone_yaw_ = m.atan2( 2*(drone_att_.w*drone_att_.z + drone_att_.x*drone_att_.y), 1-2*(drone_att_.y*drone_att_.y + drone_att_.z*drone_att_.z) )

            self.gimbal_cmd_.param1 = gimbal_euler[1] * TO_DEG - 90
            self.gimbal_cmd_.param2 = gimbal_euler[0] * TO_DEG
            self.gimbal_cmd_.param3 = gimbal_euler[2] * TO_DEG + drone_yaw_ * TO_DEG

            try:
                self.cmd_long_service.call(broadcast = False, confirmation = 0, command = 205, param1 = self.gimbal_cmd_.param1, param2 = self.gimbal_cmd_.param2, param3 = self.gimbal_cmd_.param3, param4 = 0, param5 = 0, param6 = 0, param7 = 2)

            except rospy.ServiceException, e:
                print "\nService call failed: %s"%e 

    def drone_pose_callback(self,pose):
        self.current_pose = pose
        self.has_drone_pose = True

    def target_pose_callback(self,target_pose):
        self.target_pose = target_pose
        self.has_target_pose = True

    def rotation_matrix_to_euler_angles_xyz(self, R, euler):
        if (R[2,0] < 1):

            # print m.atan2(R[2,1],R[2,2])
            # print "oi\n"
            if (R[2,0] > -1):
                euler[1] = m.asin(-R[2,0])
                euler[2] = m.atan2(R[1,0],R[0,0])
                euler[0] = m.atan2(R[2,1],R[2,2])
            else:
                euler[1] = np.pi/2
                euler[2] = -m.atan2(-R[1,2],R[1,1])
                euler[0] = 0
        else: 
            euler[1] = -np.pi/2
            euler[2] = m.atan2(-R[1,2],R[1,1])
            euler[0] = 0

    def set_mount_mode_parameters(self):
        mode_in_value = ParamValue()
        mode_out_value = ParamValue()
        mode_in_value.integer = 3
        mode_out_value.integer = 0
        last_request = rospy.Time.now()

        while not rospy.is_shutdown():
            if (rospy.Time.now() - last_request) > rospy.Duration(5.0):
                rospy.loginfo('Drone %d: setting up gimbal parameters'%self.id)
                try:
                    get_param_in_response = self.param_get_service.call("MNT_MODE_IN")
                    if get_param_in_response.success and get_param_in_response.value.integer != 3:
                        self.param_set_service.call("MNT_MODE_IN",mode_in_value)

                    get_param_out_response = self.param_get_service.call("MNT_MODE_OUT")
                    if get_param_out_response.success and get_param_out_response.value.integer != 0:
                        self.param_set_service.call("MNT_MODE_OUT",mode_out_value)

                    if get_param_in_response.value.integer == 3 and get_param_out_response.value.integer == 0:
                        break

                except rospy.ServiceException, e:
                    print "\nService call failed: %s"%e

                last_request = rospy.Time.now()
            
            time.sleep(0.2)

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
