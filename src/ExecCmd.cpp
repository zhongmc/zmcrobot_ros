#include "ros/ros.h"
#include "zmcrobot_ros/ExecCmd.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exec_cmd_clt");

    if( argc != 2)
    {
        ROS_INFO("usage: execCmd cmd");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<zmcrobot_ros::ExecCmd>("exec_cmd");
    zmcrobot_ros::ExecCmd srv;
    srv.request.cmd = argv[1];
    if( client.call(srv))
        ROS_INFO( "svc ret: %s ", srv.response.retStr.c_str() );
    else
    {
        ROS_ERROR("Failed to call exec_cmd!");
        return 1;
    }

    return 0;
     
}