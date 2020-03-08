#include "ros/ros.h"
#include "zmcrobot_ros/ExecCmd.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argc, "zmcrobot exec cmd");
    if( argc != 2)
    {
        ROS_INFO("usage: execCmd cmd");
        return 1;
    }

    ros::NodeHandle n;
    ros::serviceClient client = n.serviceClient<zmcrobot_ros::ExecCmd>("exec cmd");
    zmcrobot_ros::ExecCmd srv;
    srv.request.cmd = argv[1];
    if( client.call(srv))
        ROS_INFO( srv.response.ret);
    else
    {
        ROS_ERROR("Failed to call exec_cmd!");
        return 1;
    }

    return 0;
     
}