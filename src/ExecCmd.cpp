#include "ros/ros.h"
#include "zmcrobot_ros/ExecCmd.h"
#include <cstdlib>
#include <string.h>
#include "std_msgs/String.h"
#include <sstream>
#include <thread>

using namespace std;

bool beQuit = false;

void robotMsgCallback(const std_msgs::String::ConstPtr& msg)
{
   // ROS_INFO("I heard: [%s]", msg->data.c_str());
    cout <<  msg->data.c_str() << endl ;
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

    beQuit = true;
    cout << "msg thread terminate." <<endl;


}

void printHelp()
{
  cout << endl;
  cout << "commands: " <<endl;

   cout << "[gr;] get robot info.\n";
   cout << "[od d1,d2,d3,d4,d5;]IR distance(*1000).\n";
   cout<< "[st;]Stop robot.\n";
   cout << "[ci;] count info.\n";
   cout << "[mm pwm;]start moto 1 sec.\n";
   cout << "[sr... ]set robot params sr min_rpm,max_rpm,R,L,atObs,dfw,usafe,max_w;\n";
   cout << "[sp pwm0,pwm1,step;]speed test.\n";
   cout << "[pi type kp,ki,kd;]PID param, type 1 2 3 4.\n";
   cout << "[tl +-pwm;]turn around test +left -right.\n";
   cout << "[mg x,y;]goto goal test.\n";
   cout << "[sm 0/1;]simulate mode.\n";
   cout << "[io 0/1;]ignore obstacle mode.\n";
   cout << "[rs;]Reset robot.\n";
   cout << "[sp pwm0 pwm1 step] speed test.\n";
   cout << "[cr;] ros connect...\n";
  cout << "[si;] get robot settings and PID\n";
  cout << "[cm;] do imu calibrate.\n";
   cout << "[sc;] save imu calibtation.\n";
  cout << "[mm pwml, pwmr]  move motor  for 1min\n";
  cout << "[pm pin, mode] set pin mode; \n";
  cout << "[dw pin, value] digital write to pin.\n";
  cout << "[aw pin, value] analog write to pin.\n";
  cout << "[dr pin] digital read from pin.\n";
  cout << "[ar pin] analog read from  pin.\n";

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exec_cmd_clt");

    // if( argc != 2)
    // {
    //     ROS_INFO("usage: execCmd cmd");
    //     return 1;
    // }

    cout << "Please input cmd to exec: [Enter] to confirm!\nquit to exit, help to print cmd help info." << endl;

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
        if( beQuit )
            break;
        if( inputLine == "?")
            {
                printHelp();
                continue;
            }
        if( inputLine.length() < 2 )
            continue;           
        if( inputLine == "quit" )
            break;
        if( inputLine == "help")
        {
            printHelp();
            continue;
        }
        srv.request.cmd = inputLine;
        if( 	client.exists () )
        {
                client.call(srv );
        }
        else
        {
            cout << "service not ready, please try later......\n";
        }        
    }
    msgHandleThread.detach();
    cout << "bye!" << endl;
     return 0;    
}