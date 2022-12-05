// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int8.h>

// CONVERSIONS
#include <eigen_conversions/eigen_kdl.h>

// PSM Kinematics
#include <dvrk_rl/PSM_robot.h>
// NEEDLE Kinematics
#include "dvrk_rl/needle.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "test_kinematics");
    ros::Publisher _clikFlag_pub;
    ros::NodeHandle _nh;
    _clikFlag_pub = _nh.advertise<std_msgs::Int8>("/dvrk/Needle/clik_flag", 0);

    // Choose the frequency of the loop
    ros::Rate loop_rate = ros::Rate(1);
    ROS_INFO("test_kinematics node started");

    std_msgs::Int8 flag;
    flag.data = 7;
    while (1)
    {
        _clikFlag_pub.publish(flag);
        loop_rate.sleep();
        /*
        export ROS_MASTER_URI=http://192.168.50.144:11311
        export ROS_HOSTNAME=192.168.50.100 
        */
    }
    
    

    return 0;
}
