#!/usr/bin/env python
import rospkg
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from uav_abstraction_layer.srv import TakeOff, Land, GoToWaypoint
from uav_abstraction_layer.msg import State
from nav_msgs.msg import Odometry
import time
import signal
import sys


class FollowVehicle:

    def __init__(self,drone_id):
        self.id = drone_id
        self.current_pose = PoseStamped()
        self.desired_pose = PoseStamped()
        self.desired_pose.header.frame_id = 'map'
        self.current_vel = TwistStamped()
        self.desired_vel = TwistStamped()
        self.ual_state = State.UNINITIALIZED

        # Get relative position from params
        self.relative_position = rospy.get_param('~relative_position', default=[-5.0, 0.0, 3.0])

        # ROS Subscribers
        self.drone_pose_sub = rospy.Subscriber('ual/pose', PoseStamped, self.drone_pose_callback, queue_size=1)
        #self.drone_pose_sub = rospy.Subscriber('ual/velocity', TwistStamped, self.drone_vel_callback, queue_size=1)
        self.ual_state_sub = rospy.Subscriber('ual/state', State, self.ual_state_callback, queue_size=1)
        self.target_pose_sub = rospy.Subscriber('/drc_vehicle_xp900/odometry', Odometry, self.target_pose_callback, queue_size=1)

        # ROS Publishers
        self.set_pose_pub = rospy.Publisher('ual/set_pose', PoseStamped,queue_size=1)
        self.vel_pub = rospy.Publisher('ual/set_vel', TwistStamped, queue_size=1)

        # ROS Service clients
        takeoff_service = rospy.ServiceProxy('ual/take_off',TakeOff)
        land_service = rospy.ServiceProxy('ual/land',Land)

        # Take off when ready (UAL in LANDED_ARMED state)
        try:
            takeoff_service.wait_for_service()
            while self.ual_state != State.LANDED_ARMED and not rospy.is_shutdown():
                rospy.loginfo('Drone %d: waiting for arming'%self.id)
                time.sleep(0.2)
            time.sleep(1.0)
            takeoff_service.call(3.0, True)
        except rospy.ServiceException, e:
            print "\nService call failed: %s"%e
        
        while self.ual_state != State.FLYING_AUTO and not rospy.is_shutdown():
            rospy.loginfo('Drone %d: waiting for take off'%self.id)
            time.sleep(0.2)
        rospy.loginfo('Drone %d: ready to follow vehicle'%self.id)

        # Start timer at 30Hz to control the drone
        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)


    def timer_callback(self,event):
        #-----------------------------------------------------------------#
        # Put your code here


        #-----------------------------------------------------------------#

    def drone_pose_callback(self,pose):
        self.current_pose = pose

    def target_pose_callback(self,target_pose):
        self.target_pose = target_pose

    def ual_state_callback(self,state):
        self.ual_state = state.state

def abort_menu():
    try:
        print abort_service.call()
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e

# Finish the execution directly when Ctrl+C is pressed (signal.SIGINT received), without escalating to SIGTERM.
def signal_handler(sig, frame):
    print('Ctrl+C pressed, signal.SIGINT received.')
    sys.exit(0)


if __name__ == "__main__":
    rospy.init_node('follow_vehicle')

    drone_id = rospy.get_param('~drone_id')

    follow_vehicle = FollowVehicle(drone_id)

    # Associate signal SIGINT (Ctrl+C pressed) to handler (function "signal_handler")
    signal.signal(signal.SIGINT, signal_handler)

    # Spin while ROS is not shutdown
    rospy.spin()
