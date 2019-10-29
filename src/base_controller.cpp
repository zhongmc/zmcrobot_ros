#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include<stdio.h>
#include<algorithm>
// #include<robot_specs.h>
#include <cmath>

#include <iostream>
#include <thread>
#include <errno.h>



#include <string.h>

#include "serialport.h"

#include <signal.h>

#include <termios.h>
using namespace std;



SerialPort *m_pSerialPort  = NULL;
bool m_beQuit = false;


int kfd = 0;

void quit(int sig)

{
m_beQuit = true;
  
  (void)sig;
  ros::shutdown();

  
  exit(0);
}


void geometry_handler( char *buf , int len)
{
    cout << buf ;

}

void  IRSensor_handler( char *buf, int len )
{
    cout << buf ;
}

void IMU_handler(char *buf, int len )
{
    cout << buf ;

}

void serialReadThread(SerialPort *serial,  
      void (*geomProc)( char *,  int ), 
      void (*IRSensorProc)( char *,  int ),
      void (*IMUProc)( char *,  int ) , long serialTimeOut ) 
{
        int idx = 0;
        char buf[1024];
        char ch;

        cout << "Serial read thead started..." <<endl;
        while( true )
        {
                if( serial->available( serialTimeOut ) )
                {
                        while( true )
                        {
                            int ret = serial->read(&ch, 1 );
                            if( ret != 1 )
                                break;

                            buf[idx++] = ch;

                            if( ch == '\r' || ch == '\n')
                            {
                                buf[idx] = '\0';
                                if( idx > 2 && buf[0] == 'R' && buf[1] == 'P')
                                {
                                    if( geomProc != NULL )
                                      geomProc( buf, idx );
                                    else
                                      cout << buf;                                      
                                }
                                else if( idx > 2 && buf[0] == 'I' && buf[1]=='R')
                                {
                                    IRSensorProc(buf, idx );
                                }
                                else if(  idx>2 && buf[0] == 'I' && buf[1] == 'M')
                                {
                                    IMUProc( buf, idx );
                                }
                                else
                                {
                                    cout << buf;
                                }
                                idx = 0;
                            }

                           if( idx > 1020 )
                           {
                               cout << buf << endl;
                               cout << "read out of buff !" << endl;
                               idx = 0;

                           } 
                        }
                 }


               if( m_beQuit  )
               {
                 cout << "required to quit, close serial thread!" << endl;
                  break;
               }

        }
}



ros::Time current_time;



void handle_twist( const geometry_msgs::Twist& cmd_msg) {
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;

  char cmd[100];
  memset(cmd, 0, 30);
  sprintf(cmd, "sd%.4f,%.4f\n", x, z);
  if(  m_pSerialPort != NULL )
      m_pSerialPort->write(cmd, strlen( cmd )); 

  cout<<"send cmd:" << cmd ;
}

int main(int argc, char** argv){

ros::init(argc, argv, "base_controller");

ros::NodeHandle  nh;

ros::NodeHandle nh_private_("~");

ros::Subscriber sub = nh.subscribe("cmd_vel", 50,  handle_twist );
ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

tf::TransformBroadcaster broadcaster;


    long  baudrate = 115200;
  string serialPath = "/dev/ttyACM0";
    // if( argc>2 )
    //     baudrate = atoi( argv[2]);

    SerialPort::OpenOptions options = SerialPort::defaultOptions;
    options.baudRate = SerialPort::BaudRateMake( baudrate );

    cout << "try to open port " <<  serialPath << " at baudrate of " << baudrate << endl;

    SerialPort serialPort(serialPath, options );


    bool ret = serialPort.isOpen();

    if( !ret )
    {
        cout << "failed to open...\n";
        cout << errno <<  ": " <<strerror(errno) <<endl;
        return -1;
    }

    m_pSerialPort = &serialPort;

    int idx;
    char buf[200];

     std::thread readThread(serialReadThread,  &serialPort,  
                  &geometry_handler, &IRSensor_handler, &IMU_handler , 1000000 );


  double rate = 20.0;

  signal(SIGINT,quit);

ros::Rate r(rate);
  while(nh.ok()){
    ros::spinOnce();
    // ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("rpm", n, d);
    current_time = ros::Time::now();
  }

  m_beQuit = true;
  serialPort.close();
}