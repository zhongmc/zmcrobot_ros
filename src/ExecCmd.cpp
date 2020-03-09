#include "ros/ros.h"
#include "zmcrobot_ros/ExecCmd.h"
#include <cstdlib>

#include "std_msgs/String.h"



void robotMsgCallback(const std_msgs::String::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%s]", msg->data.c_str());
    cout <<  msg->data.c_str() ;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exec_cmd_clt");

    if( argc != 2)
    {
        ROS_INFO("usage: execCmd cmd");
        return 1;
    }

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("robot_msg", 100, robotMsgCallback);


    ros::ServiceClient client = n.serviceClient<zmcrobot_ros::ExecCmd>("exec_cmd");
    zmcrobot_ros::ExecCmd srv;
    srv.request.cmd = argv[1];

    string inputLine;

    while(true )
    {
        getline(cin, inputLine );

        if( inputLine == "quit" )
            break;

        srv.request.cmd = inputLine;
        if( !client.call(srv))
        {
            ROS_ERROR("Failed to call exec_cmd!");
            return 1;
        }
    }

     return 0;
     
}