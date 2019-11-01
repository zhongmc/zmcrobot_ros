#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/PointCloud.h>
#include <stdio.h>
#include <algorithm>
// #include<robot_specs.h>
#include <cmath>
#include <iostream>
#include <thread>
#include <errno.h>
#include <string.h>
#include "serialport.h"
#include <signal.h>
#include <termios.h>

#include "IRSensor.h"

using namespace std;

int getIntsFromStr(int *ints, const char *buf, int count);

class SerialMsgHandler
{
public:
  SerialMsgHandler(SerialPort *pSerialPort);

  void serialMsgLoop(long timeOut);
  bool m_beQuit;

private:
  SerialPort *m_pSerialPort;

  ros::NodeHandle nh_;
  ros::Publisher odom_pub;
  ros::Publisher cloud_pub;
  tf::TransformBroadcaster broadcaster;

  const char *base_link; // "/base_link";
  const char *odom;      // [] = "/odom";

  IRSensor *irSensors[5];

private:
  void geometry_handle(char *buf, int len);
  void IRSensor_handle(char *buf, int len);
  void IMU_handle(char *buf, int len);
  void publishGeometryMsg(double x, double y, double theta, double w, double v);
};

SerialMsgHandler::SerialMsgHandler(SerialPort *pSerialPort) : m_beQuit(false),
                                                              base_link("/base_link"),
                                                              odom("/odem")
{
  m_pSerialPort = pSerialPort;
  odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 50);
  cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("cloud", 50);

  irSensors[0] = new IRSensor(-0.073, 0.066, M_PI / 2, GP2Y0A21);
  irSensors[1] = new IRSensor(0.061, 0.05, M_PI / 4, GP2Y0A21); // 0.16,0.045, PI/6 0.075, 0.035
  irSensors[2] = new IRSensor(0.072, 0.0, 0, GP2Y0A21);
  irSensors[3] = new IRSensor(0.061, -0.05, -M_PI / 4, GP2Y0A21);
  irSensors[4] = new IRSensor(-0.073, -0.066, -M_PI / 2, GP2Y0A21);
}

void SerialMsgHandler::serialMsgLoop(long timeOut)
{

  int idx = 0;
  char buf[1024];
  char ch;

  cout << "Serial read thead started..." << endl;
  while (true)
  {
    if (m_pSerialPort->available(timeOut))
    {
      while (true)
      {
        int ret = m_pSerialPort->read(&ch, 1);
        if (ret != 1)
          break;

        buf[idx++] = ch;

        if (ch == '\r' || ch == '\n')
        {
          buf[idx] = '\0';
          if (idx > 2 && buf[0] == 'R' && buf[1] == 'P')
          {

            geometry_handle(buf, idx);
          }
          else if (idx > 2 && buf[0] == 'I' && buf[1] == 'R')
          {
            IRSensor_handle(buf, idx);
          }
          else if (idx > 2 && buf[0] == 'I' && buf[1] == 'M')
          {
            IMU_handle(buf, idx);
          }
          else
          {
            cout << buf;
          }
          idx = 0;
        }

        if (idx > 1020)
        {
          cout << buf << endl;
          cout << "read out of buff !" << endl;
          idx = 0;
        }
      }
    }

    if (m_beQuit)
    {
      cout << "required to quit, close serial thread!" << endl;
      break;
    }
  }
}

void SerialMsgHandler::geometry_handle(char *buf, int len)
{
  //"RPx,y,theta,v" x=x*10000,y=y*1000,theta=theta*10000,v=%04f

  double x, y, theta, w, v;
  int ints[5];

  int ret = getIntsFromStr(ints, buf + 2, 5);
  if (ret != 5)
  {
    cout << "geometry error:" << ret << buf;
    return;
  }

  x = (double)ints[0] / 10000.0;
  y = (double)ints[1] / 10000.0;
  theta = (double)ints[2] / 10000.0;
  w = (double)ints[3] / 10000.0;
  v = (double)ints[4] / 10000.0;
  publishGeometryMsg(x, y, theta, w, v);
}

void SerialMsgHandler::publishGeometryMsg(double x, double y, double theta, double w, double v)
{
  ros::Time current_time;
  current_time = ros::Time::now();

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

  geometry_msgs::TransformStamped t;
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = 0.0;
  t.transform.rotation = odom_quat;
  t.header.stamp = current_time;

  broadcaster.sendTransform(t);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = odom;
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.child_frame_id = base_link;
  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = w;

  odom_pub.publish(odom_msg);
}

void SerialMsgHandler::IRSensor_handle(char *buf, int len)
{
  // cout << buf;
  int ints[5];

  int ret = getIntsFromStr(ints, buf + 2, 5);
  if (ret != 5)
  {
    cout << "ir data error:" << ret << buf;
    return;
  }

  int num_points = 5;
  sensor_msgs::PointCloud cloud;
  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = "sensor_frame";
  cloud.points.resize(num_points);
  cloud.channels.resize(1);
  cloud.channels[0].name = "intensities";
  cloud.channels[0].values.resize(num_points);

  for (unsigned int i = 0; i < num_points; ++i)
  {

    irSensors[i]->setDistance(ints[i] / 100.0);
    cloud.points[i].x = irSensors[i]->x;
    cloud.points[i].y = irSensors[i]->y;
    cloud.points[i].z = 0.1;
    cloud.channels[0].values[i] = 10; //???
  }

  cloud_pub.publish(cloud);
}

void SerialMsgHandler::IMU_handle(char *buf, int len)
{
  cout << buf;
}

void serialMsgThread(SerialMsgHandler *pHandler, long timeout)
{
  pHandler->serialMsgLoop(timeout);
}

void serialReadThread(SerialPort *serial,
                      void (*geomProc)(char *, int),
                      void (*IRSensorProc)(char *, int),
                      void (*IMUProc)(char *, int), long serialTimeOut)
{
  int idx = 0;
  char buf[1024];
  char ch;

  cout << "Serial read thead started..." << endl;
  while (true)
  {
    if (serial->available(serialTimeOut))
    {
      while (true)
      {
        int ret = serial->read(&ch, 1);
        if (ret != 1)
          break;

        buf[idx++] = ch;

        if (ch == '\r' || ch == '\n')
        {
          buf[idx] = '\0';
          if (idx > 2 && buf[0] == 'R' && buf[1] == 'P')
          {
            if (geomProc != NULL)
              geomProc(buf, idx);
            else
              cout << buf;
          }
          else if (idx > 2 && buf[0] == 'I' && buf[1] == 'R')
          {
            IRSensorProc(buf, idx);
          }
          else if (idx > 2 && buf[0] == 'I' && buf[1] == 'M')
          {
            IMUProc(buf, idx);
          }
          else
          {
            cout << buf;
          }
          idx = 0;
        }

        if (idx > 1020)
        {
          cout << buf << endl;
          cout << "read out of buff !" << endl;
          idx = 0;
        }
      }
    }

    //  if( m_beQuit  )
    //  {
    //    cout << "required to quit, close serial thread!" << endl;
    //     break;
    //  }
  }
}

SerialPort *m_pSerialPort = NULL;

void handle_twist(const geometry_msgs::Twist &cmd_msg)
{
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;

  char cmd[100];
  memset(cmd, 0, 30);
  sprintf(cmd, "sd%.4f,%.4f\n", x, z);
  if (m_pSerialPort != NULL)
    m_pSerialPort->write(cmd, strlen(cmd));

  cout << "send cmd:" << cmd;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "base_controller");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("cmd_vel", 50, handle_twist);

  int baud = 115200;
  string port = "/dev/ttyACM0";
  bool simulateMode = true;
  int timeout = 500000;

  nh.param("baud", baud, baud);
  nh.param("port", port, port);
  nh.param("simulate", simulateMode, simulateMode);
  nh.param("timeout", timeout, timeout);

  // if( argc>2 )
  //     baudrate = atoi( argv[2]);

  SerialPort::OpenOptions options = SerialPort::defaultOptions;
  options.baudRate = SerialPort::BaudRateMake(baud);

  cout << "try to open port " << port << " at baudrate of " << baud << endl;

  SerialPort serialPort(port, options);

  m_pSerialPort = &serialPort;

  bool ret = serialPort.isOpen();

  if (!ret)
  {
    cout << "failed to open...\n";
    cout << errno << ": " << strerror(errno) << endl;
    return -1;
  }

  if (simulateMode == true)
  {
    cout << "set to simulate mode ...\n";
    serialPort.write("\nsm1\n", 5);
  }

  int idx;
  char buf[200];

  SerialMsgHandler msgHandler(&serialPort);

  //start the serial msg read and handle thread
  std::thread msgHandleThread(serialMsgThread, &msgHandler, timeout);

  //  std::thread readThread(serialReadThread,  &serialPort,
  //               &geometry_handler, &IRSensor_handler, &IMU_handler , 1000000 );

  double rate = 20.0;
  ros::Rate r(rate);
  ros::Time current_time;

  // signal(SIGINT,quit);

  while (nh.ok())
  {
    ros::spinOnce();
    // ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("rpm", n, d);
    current_time = ros::Time::now();

    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.0, 0.0)),
            ros::Time::now(), "base_link", "sensor_frame"));
  }

  msgHandler.m_beQuit = true;
  serialPort.close();
}

int getIntsFromStr(int *ints, const char *buf, int count)
{
  if (ints == NULL || buf == NULL)
    return 0;
  int cnt = 0;
  const char *p = buf;
  while (cnt < count)
  {
    ints[cnt++] = atoi(p);
    p = strchr(p, ',');
    if (p == NULL)
      return cnt;
    p++;
  }
  return cnt;
}
