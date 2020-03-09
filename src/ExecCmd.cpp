#include "ros/ros.h"
#include "zmcrobot_ros/ExecCmd.h"
#include <cstdlib>
#include <string.h>
#include "std_msgs/String.h"
#include <sstream>
#include <thread>

using namespace std;


void robotMsgCallback(const std_msgs::String::ConstPtr& msg)
{
   // ROS_INFO("I heard: [%s]", msg->data.c_str());
    cout <<  msg->data.c_str() ;
}


void msgThread()
{

    cout << "start msg thread..." <<endl;
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("robot_msg", 100, robotMsgCallback);

    while(ros::ok())
    {
        ros::spinOnce();
    }

    cout << "msg thread terminate." <<endl;


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exec_cmd_clt");

    // if( argc != 2)
    // {
    //     ROS_INFO("usage: execCmd cmd");
    //     return 1;
    // }

    cout << "Please input cmd to exec: [Enter] to confirm!" << endl;

 std::thread msgHandleThread(msgThread);

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<zmcrobot_ros::ExecCmd>("exec_cmd");
    zmcrobot_ros::ExecCmd srv;
    string inputLine;
    while(true )
    {
        getline(cin, inputLine );

        if( inputLine.empty() )
            break;
        if( inputLine.length() < 2 )
            continue;
            
        if( inputLine == "quit" )
            break;

        srv.request.cmd = inputLine;
        if( !client.call(srv))
        {
            ROS_ERROR("Failed to call exec_cmd!");
            return 1;
        }
    }

    cout << "finished!" << endl;

     return 0;
     
}