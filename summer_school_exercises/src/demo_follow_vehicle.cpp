#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include <eigen_conversions/eigen_msg.h>

#include <uav_abstraction_layer/ual.h>

class DemoFollowVehicle
{
private:

  ros::ServiceClient cmd_long_client_;
  ros::ServiceClient set_param_client_;
  ros::ServiceClient get_param_client_;

  ros::Timer timer_;
  
  geometry_msgs::Point        vehicle_pos_msg_;
  geometry_msgs::Quaternion   vehicle_att_msg_;
  geometry_msgs::PoseStamped  setpoint_pose_msg_;

  Eigen::Vector3d     vehicle_pos_;
  Eigen::Quaterniond  vehicle_att_;
  Eigen::Vector3d     drone_pos_;
  Eigen::Vector3d     drone_init_pos_;

  mavros_msgs::CommandLong gimbal_cmd_;

  grvc::ual::UAL ual_;
  int uav_id_;

  int drone_id;
  std::string ns;

  // Timer callback
  void timerCallback(const ros::TimerEvent&)
  {
    // update drone pose
    tf::pointMsgToEigen(ual_.pose().pose.position, drone_pos_);

    // send drone to vehicle position
    setpoint_pose_msg_.header.stamp = ros::Time::now();
    setpoint_pose_msg_.header.frame_id = "map";
    setpoint_pose_msg_.pose.position = vehicle_pos_msg_;
    setpoint_pose_msg_.pose.position.x += drone_init_pos_.x(); // avoid being at azimuth and inconsistent roll angle of the gimbal camera
    setpoint_pose_msg_.pose.position.y += drone_init_pos_.y();
    setpoint_pose_msg_.pose.position.z = 6.0;
    setpoint_pose_msg_.pose.orientation = vehicle_att_msg_;
    ual_.goToWaypoint(setpoint_pose_msg_,false);

    // compute gimbal orientation to point to vehicle
    Eigen::Vector3d pos_vehicle_drone = vehicle_pos_ - drone_pos_;
    pos_vehicle_drone /= pos_vehicle_drone.norm();

    Eigen::Vector3d rot_axis = pos_vehicle_drone.cross(Eigen::Vector3d::UnitX());
    double angle = std::acos(pos_vehicle_drone.dot(Eigen::Vector3d::UnitX()));
    Eigen::Matrix3d rot = Eigen::AngleAxisd(angle, rot_axis).toRotationMatrix();
    Eigen::Vector3d euler;
    toEulerAngles(Eigen::Quaterniond(rot), euler);

    gimbal_cmd_.request.param1 = euler.y()*180/M_PI;  // pitch
    gimbal_cmd_.request.param2 = euler.x()*180/M_PI*0;// roll
    gimbal_cmd_.request.param3 = euler.z()*180/M_PI;  // yaw
    cmd_long_client_.call(gimbal_cmd_);
  }

  // DRC XP900 vechicle's odometry callback
  void vehicle_odom(const nav_msgs::Odometry::ConstPtr& msg)
  {
    vehicle_pos_msg_ = msg->pose.pose.position;
    vehicle_att_msg_ = msg->pose.pose.orientation;

    tf::pointMsgToEigen(msg->pose.pose.position, vehicle_pos_);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation, vehicle_att_);

    vehicle_att_msg_.x = 0;
    vehicle_att_msg_.y = 0;
    vehicle_att_msg_.w = cos(atan2(vehicle_att_msg_.z, vehicle_att_msg_.w));
  }

  void setMountModeParameters()
  {
    mavros_msgs::ParamGet get_param_in, get_param_out;
    get_param_in.request.param_id = "MNT_MODE_IN";
    get_param_out.request.param_id = "MNT_MODE_OUT";

    mavros_msgs::ParamSet set_param;

    ros::Time last_request = ros::Time::now();
    while(ros::ok())
    {
      if((ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if(get_param_client_.call(get_param_in) && get_param_in.response.success && get_param_in.response.value.integer != 3)
        {
          set_param.request.param_id = "MNT_MODE_IN";
          set_param.request.value.integer = 3;
          set_param_client_.call(set_param);
        }

        if(get_param_client_.call(get_param_out) && get_param_out.response.success && get_param_out.response.value.integer != 0)
        {
          set_param.request.param_id = "MNT_MODE_OUT";
          set_param.request.value.integer = 0;
          set_param_client_.call(set_param);
        }

        if(get_param_in.response.value.integer == 3 &&
           get_param_out.response.value.integer == 0)
          break;

        last_request = ros::Time::now();
      }

      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
  }

  void toEulerAngles(const Eigen::Quaterniond& q, Eigen::Vector3d& euler)
  {
    // roll (x-axis rotation)
    double t0 = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double t1 = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    euler.x() = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = 2.0 * (q.w() * q.y() - q.z() * q.x());
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    euler.y() = std::asin(t2);

    // yaw (z-axis rotation)
    double t3 = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double t4 = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
    euler.z() = std::atan2(t3, t4);
  }

public:
  // Constructor
  DemoFollowVehicle(int _argc, char** _argv) : ual_(_argc, _argv)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Initialise UAL
    while (!ual_.isReady() && ros::ok()) {
        sleep(1);
    }
    pnh.param<int>("uav_id", uav_id_, 1);
    ROS_INFO("DemoFollowVehicle [%d] initialized!", uav_id_);

    // Initialise gimbal cmd
#ifdef MAVROS_VERSION_BELOW_0_25_0
    gimbal_cmd_.request.command = mavros_msgs::CommandCode::CMD_DO_MOUNT_CONTROL;
#else
    gimbal_cmd_.request.command = mavros_msgs::CommandCode::DO_MOUNT_CONTROL;
#endif
    gimbal_cmd_.request.confirmation = false;
    gimbal_cmd_.request.param7 = 2; // MAV_MOUNT_MODE::MAV_MOUNT_MODE_MAVLINK_TARGETING

    // Initialise sp pose msg
    setpoint_pose_msg_.header.frame_id = "base_footprint";
    setpoint_pose_msg_.pose.position.x = 0;
    setpoint_pose_msg_.pose.position.y = 0;
    setpoint_pose_msg_.pose.position.z = 2.5;

    // Subscribers
    ros::Subscriber vechile_odometry_sub = nh.subscribe<nav_msgs::Odometry>
      ("drc_vehicle_xp900/odometry", 10, &DemoFollowVehicle::vehicle_odom, this);

    // Services
    cmd_long_client_ = nh.serviceClient<mavros_msgs::CommandLong>("drone_"+std::to_string(uav_id_)+"/mavros/cmd/command");
    set_param_client_ = nh.serviceClient<mavros_msgs::ParamSet>("drone_"+std::to_string(uav_id_)+"/mavros/param/set");
    get_param_client_ = nh.serviceClient<mavros_msgs::ParamGet>("drone_"+std::to_string(uav_id_)+"/mavros/param/get");

    // start Timer
    timer_ = nh.createTimer(ros::Duration(0.05), &DemoFollowVehicle::timerCallback, this);

    ROS_WARN("Started Following Vehicle example.");

    // Wait for 5 seconds to let the Gazebo GUI show up.
    ROS_WARN("Waiting to let Gazebo initialise...");
    ros::Duration(2.0).sleep();

    // Set initial drone pose
    tf::pointMsgToEigen(ual_.pose().pose.position, drone_pos_);
    drone_init_pos_ = drone_pos_;

    setMountModeParameters();

    // Takeoff
    ual_.takeOff(6.0);

    // Set MNT_MODE parameters on FCU for gimbal control via Mavlink
    
    ROS_INFO("Ready to follow vehicle.");

    // ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    // spinner.spin();
    //ros::spin();
    while (ros::ok())
      sleep(1);
  }

  // Destructor
  ~DemoFollowVehicle(){};
};

int main(int _argc, char **_argv)
{
  ros::init(_argc, _argv, "demo_follow_vehicle");
  int uav_id;
  ros::param::param<int>("~uav_id", uav_id, 1);
  ROS_INFO("Setting up DemoFollowVehicle[%d]", uav_id);
  // Setting arguments
  ros::param::set("~ns_prefix", "drone_");
  ros::param::set("~pose_frame_id", "map");
  sleep(10);
  DemoFollowVehicle demo_follow_vehicle(_argc,_argv);
  return 0;
}
