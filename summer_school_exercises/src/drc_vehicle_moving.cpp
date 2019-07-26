#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"drc_vehicle_moving");
    ros::NodeHandle nh;
    ros::Publisher vehicle_speed_pub;
    ros::Publisher attitude_pub;

    // Publishers
    vehicle_speed_pub = nh.advertise<std_msgs::Float64>("drc_vehicle_xp900/speed/cmd", 1);

    // Wait until key pressed
    std::cout << "Press any key to start moving the vehicle.\n";
    char c;
    std::cin >> c;

    // send speed cmd to xp900 vehicle
    std_msgs::Float64 speed_msg;
    speed_msg.data = 2.0;
    vehicle_speed_pub.publish(speed_msg);

    ros::spin();

    return 0;
}
