#include <ros/ros.h>
#include <ros/console.h>
#include <string.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


#include <stdio.h>
#include <algorithm>
// #include<robot_specs.h>
#include <cmath>
#include <iostream>
#include <thread>
#include <errno.h>
#include <signal.h>
#include <sstream>
#include <fstream>


nav_msgs::Path path;
ros::Publisher path_pub;

double cur_x, cur_y;

void odomCallback(const nav_msgs::OdometryConstPtr& msg){
 
    ros::Time current_time = ros::Time::now();   
    path.header.stamp=current_time;
    geometry_msgs::PoseStamped this_pose_stamped;
        
        double x =  msg->pose.pose.position.x;
        double y =  msg->pose.pose.position.y;

        double dx  = x - cur_x;
        double dy = y - cur_y;

        if(  (dx*dx + dy*dy) == 0  )
            return;

        cur_x = x;
        cur_y = y; 

    this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
    this_pose_stamped.pose.position.y =  msg->pose.pose.position.y;

    this_pose_stamped.pose.orientation = msg->pose.pose.orientation;
    
    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="odom";
    path.poses.push_back(this_pose_stamped);

    path.header.stamp = current_time;
    path.header.frame_id = "odom";
    path_pub.publish(path);

}

main (int argc, char **argv)
{

    ros::init (argc, argv, "OdomPath");
    ros::NodeHandle nlh("~");

    std::string odomTopic =  "odom";
    std::string pathTopic = "odom_path";

     nlh.param("odom_topic", odomTopic, odomTopic);
    nlh.param("path_topic", pathTopic, pathTopic);

    std::cout<<"publish odom: " << odomTopic << "  to  path: " << pathTopic << std::endl;
 ROS_INFO("publish odom: %s  to path: %s", odomTopic.c_str(),  pathTopic.c_str() );
ros::NodeHandle nh;

ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(odomTopic, 1, odomCallback);
path.header.frame_id="odom";
path_pub = nh.advertise<nav_msgs::Path>(pathTopic, 1,  true);

cur_x = 0;
cur_y = 0;

ros::Rate loop_rate(1);
while (ros::ok())
{

ros::spinOnce(); // check for incoming messages
loop_rate.sleep();
}

return 0;
}