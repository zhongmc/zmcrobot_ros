#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

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
#include "yaml-cpp/yaml.h"
#include "zmcrobot_ros/ExecCmd.h"
#include "std_msgs/String.h"
#include <sstream>
#include <fstream>


using namespace std;

int getIntsFromStr(int *ints, const char *buf, int count);
int getDoublesFromStr(double *dins, const char *buf, int count );


 int byteToInt16(char *data, int offset )
	{
		int value = ((data[offset+1] & 0x7f)<<8) | (data[offset] & 0xff);
		
		if( (data[offset+1] & 0x80) != 0  )
		{
			value = -value;
		}
		return value;
	}

 int byteToInt32(char *data, int offset )
	{
		int value = ((data[offset+3] & 0x7f)<<24) | (data[offset+2] & 0xff << 16)| (data[offset+1] & 0xff << 8) | (data[offset] & 0xff);
		
		if( (data[offset+3] & 0x80) != 0  )
		{
			value = -value;
		}
		return value;
	}


class SerialMsgHandler
{
public:
  SerialMsgHandler(SerialPort *pSerialPort);

  void serialMsgLoop(long timeOut);
  bool m_beQuit;

  void loadCalibrateParams(string fileName);
  void saveCalibration();

  void setRecordData(bool val );
  void recordSerialData(char *buf, int len );
  void close();
//publish the odom to baselink trasfer if true
  bool m_publish_tf;
  bool m_record_data;

private:
  SerialPort *m_pSerialPort;
   
   std::ofstream m_fout;
  ros::NodeHandle nh_;
  ros::Publisher odom_pub;
  ros::Publisher cloud_pub;
  ros::Publisher imu_pub;

  ros::Publisher  imu_raw_pub;
  ros::Publisher imu_mag_pub;

  ros::Publisher robot_msg_pub;

  tf::TransformBroadcaster broadcaster;

  const char *base_link; // "/base_link";
  const char *odom;      // [] = "/odom";

  IRSensor *irSensors[5];

  string calFileName; 

  double aRes, gRes, mRes;

  double accelBias[3], gyroBias[3], magBias[3];
  double magScale[3];
  double magCalibration[3];    //onboard calibration of mag

  bool inBinaryPkg;
  int binaryPkgLen;

private:
  void comDataReaded(char *buf, int len );
  void geometry_handle(char *buf, int len);
  void IRSensor_handle(char *buf, int len);
  void IMU_handle(char *buf, int len);
  void calibDataHandle(char *buf, int len );

  void BinaryComDataReaded(char *buf, int len );
  void IMU_RawDataHandle(char *buf, int len );

  void IMURawDataHandle(char *buf, int len);
  void RobotStateHandle(char *buf, int len );

  
  void publishGeometryMsg(double x, double y, double theta, double w, double v);
};


void SerialMsgHandler::close()
{
  m_beQuit = true;
  if( m_record_data )
    m_fout.close();

    cout << "close the msg handler!" << endl;
}

SerialMsgHandler::SerialMsgHandler(SerialPort *pSerialPort) : m_beQuit(false), m_publish_tf(false), 
                                                            m_record_data(false),
                                                              base_link("base_link"),   //base_footprint
                                                              odom("odom")
{
  m_pSerialPort = pSerialPort;
  odom_pub = nh_.advertise<nav_msgs::Odometry>("odom_arduino", 50);
  cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("cloud", 50);
  imu_pub = nh_.advertise<sensor_msgs::Imu>("imu_arduino", 20);

  imu_raw_pub = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
  imu_mag_pub = nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
  
  robot_msg_pub = nh_.advertise<std_msgs::String>("robot_msg", 50);

  irSensors[0] = new IRSensor(-0.073, 0.066, M_PI / 2, GP2Y0A21);
  irSensors[1] = new IRSensor(0.061, 0.05, M_PI / 4, GP2Y0A21); // 0.16,0.045, PI/6 0.075, 0.035
  irSensors[2] = new IRSensor(0.072, 0.0, 0, GP2Y0A21);
  irSensors[3] = new IRSensor(0.061, -0.05, -M_PI / 4, GP2Y0A21);
  irSensors[4] = new IRSensor(-0.073, -0.066, -M_PI / 2, GP2Y0A21);

  aRes = 0.000061035;
  gRes = 0.0076294;
  mRes = 5.997558;

  for( int i=0; i<3; i++)
  {
    accelBias[i] = 0;
    gyroBias[i] = 0;
    magBias[i] = 0;
    magScale[i] = 1;
    magCalibration[i] = 1;
  }

  inBinaryPkg = false;
  binaryPkgLen = -1;

}


void SerialMsgHandler::recordSerialData(char *buf, int len )
{
    if( !m_record_data )
      return;
      // m_fout << "==" << len << inBinaryPkg << binaryPkgLen << ":";
      m_fout.write(buf, len);
}

void SerialMsgHandler::setRecordData(bool val )
{
   m_record_data = val;
   if( val )
   {
     try{
       m_fout.open("/home/zhongmc/serial_data.bin", ios::out | ios::binary);
       cout << "Open the serial data file ..." <<  endl;
   }
   catch(...)
   {

   }
   }
}

void SerialMsgHandler::saveCalibration()
{

  
  YAML::Node node;

    node["res"].push_back( aRes );
    node["res"].push_back( gRes );
    node["res"].push_back( mRes );

   for (int i = 0; i < 3; i++)
    {
      node["AccelBias"].push_back( accelBias[i] );
      node["GyroBias"].push_back( gyroBias[i] );
      node["MagBias"].push_back( magBias[i] );
      node["Scale"].push_back( magScale[i] );
      node["MagCalibration"].push_back( magCalibration[i] );
    }

  try
  {
    std::ofstream fout;
    fout.open(calFileName );
    fout << node;
    fout.close();
  }
  catch (...)
  {
    return ;
  }

}


void SerialMsgHandler::loadCalibrateParams(string fileName )
{

  try
  {
    calFileName = fileName;
    cout << "Load calib file " << fileName << endl ;
    YAML::Node node = YAML::LoadFile(fileName);
    assert(node["res"].IsSequence() && node["res"].size() == 3);
    assert(node["AccelBias"].IsSequence() && node["AccelBias"].size() == 3);
    assert(node["GyroBias"].IsSequence() && node["GyroBias"].size() == 3);
    assert(node["MagBias"].IsSequence() && node["MagBias"].size() == 3);
    assert(node["Scale"].IsSequence() && node["Scale"].size() == 3);
    assert( node["MagCalibration"].IsSequence() && node["MagCalibration"].size() == 3);

    aRes = node["res"][0].as<double>();
    gRes = node["res"][1].as<double>();
    mRes = node["res"][2].as<double>();

    cout << aRes << ", " << gRes << ", " << mRes << endl;

    for (int i = 0; i < 3; i++)
    {
      accelBias[i] = node["AccelBias"][i].as<double>();
      gyroBias[i] = node["GyroBias"][i].as<double>();
      magBias[i] = node["MagBias"][i].as<double>();
      magScale[i] =  node["Scale"][i].as<double>();
      magCalibration[i] = node["MagCalibration"][i].as<double>();
    }

   cout << "accel bias: " << accelBias[0] << ",  " <<  accelBias[1] << ",  "  << accelBias[2] << endl;
   cout << "gyro bias: " << gyroBias[0] << ",  " <<  gyroBias[1] << ",  "  << gyroBias[2] << endl;
   cout << "mag  bias: " << magBias[0] << ",  " <<  magBias[1] << ",  "  << magBias[2] << endl;
   cout << "mag  scale: " << magScale[0] << ",  " <<  magScale[1] << ",  "  << magScale[2] << endl;
   cout << "mag  calibration: " << magCalibration[0] << ",  " <<  magCalibration[1] << ",  "  << magCalibration[2] << endl;


  }
  catch (...)
  {
    cout << "failed to load the calib file..." << endl;
    return;
  }

}




void SerialMsgHandler::serialMsgLoop(long timeOut)
{

  int idx = 0;
  char comBuffer[1024];
  char ch;
  int bufOff = 0;
 
  unsigned char readByte;
  cout << "Serial read thead started..." << endl;
  while (true)
  {

    if (m_pSerialPort->available(timeOut))
    {
      int ret = m_pSerialPort->read(&readByte, 1);
      if( ret <= 0 )
      {
        ROS_INFO("Error read serial: %d", ret );
          continue;
      }
      
      if( inBinaryPkg )
      {
        if( binaryPkgLen == -1 )
        {
          binaryPkgLen = readByte;
          if( binaryPkgLen <= 0  || binaryPkgLen > 120  )
          {
              bufOff = 0;
              inBinaryPkg = false;
              binaryPkgLen = -1;
          }
          else
            comBuffer[bufOff++] = readByte;
          continue;
        }
        comBuffer[bufOff++] = readByte;
        if( bufOff >= (binaryPkgLen + 2) ) //full pkg readed
        {
          recordSerialData(comBuffer, bufOff );
          BinaryComDataReaded(comBuffer, bufOff);
          bufOff = 0;
          inBinaryPkg = false;
          binaryPkgLen = -1;
        }
        continue;
      }

      if( readByte == 0xA0 || readByte == 0xA1 || readByte == 0xA2 || readByte == 0xA3 || readByte == 0xA4 ) //binary package
      {
        bufOff = 0;
        inBinaryPkg = true;
        binaryPkgLen = -1;
        comBuffer[bufOff++] = readByte;
          continue;
      }

        comBuffer[bufOff++] = readByte;
      if (readByte == '\r' || readByte == '\n'  || readByte == ';') {
                  recordSerialData(comBuffer, bufOff );
                  comDataReaded(comBuffer, bufOff);
        bufOff = 0;
        inBinaryPkg = false;
        binaryPkgLen = -1;
        continue;
      }
      if (bufOff >= 1020) {
          recordSerialData(comBuffer, bufOff );
          ROS_INFO("Error Data: %s", comBuffer );
          bufOff = 0;
        inBinaryPkg = false;
        binaryPkgLen = -1;
      }
 
    }
    if (m_beQuit)
    {
      cout << "required to quit, close serial thread!" << endl;
      break;
    }

  }
  if( m_record_data )
    m_fout.close();

}


void SerialMsgHandler::comDataReaded(char *buf, int len )
{
  if( len < 2 )
  {
   // ROS_INFO("com date err:  %s", buf);
    return;
  }

  if ( buf[0] == 'R' && buf[1] == 'P')
  {
    geometry_handle(buf, len);
  }
  else if (buf[0] == 'I' && buf[1] == 'R')
  {
    IRSensor_handle(buf, len);
  }
  else if (buf[0] == 'I' && buf[1] == 'M')
  {
    IMU_handle(buf, len);
  }
  else if( buf[0] == 'R' && buf[1] == 'D' ) //IMU raw data
  {
    IMU_RawDataHandle(buf, len );
  }
  else if( buf[0] == 'R' && buf[1] == 'E')  //ready
  {
    ROS_INFO("Robot ready, send connect cmd cr;");
            // cout << "robot ready!" << endl;
            // cout << "send connect cmd CR..." << endl;
    m_pSerialPort->write( "\ncr;\n" , 4);
  }
  else if( buf[0] == 'C' && buf[1] == 'M')
  {
    ROS_INFO("cal data... %s", buf);
    calibDataHandle(buf, len);
  }
  else
  {
     buf[len]  = 0;
    ROS_INFO("robot: %s", buf);
            // cout << idx << ": " <<  buf;
    std_msgs::String msg;
    string msgStr = buf;
    msg.data  = msgStr;
    robot_msg_pub.publish( msg );
  }
}

void SerialMsgHandler::BinaryComDataReaded(char *buf, int len )
{
    // recordSerialData("-BIN-", 5);
    // recordSerialData(buf, 2);

  unsigned char pkg = (unsigned char ) buf[0];

    if(pkg == 0xA0 || pkg == 0xA1)
    {
      recordSerialData("-ST-", 4);
       RobotStateHandle(buf, len);
    }
    else if( pkg == 0xA2 )
    {
      recordSerialData("-IMU-", 5);
    IMURawDataHandle(buf, len);
    }
}


  void SerialMsgHandler::IMURawDataHandle(char *buf, int len)
  {
      int16_t intValue[9];
      uint8_t *raw = (uint8_t *)buf;

      for( int i=0; i<len; i+=2 )
        intValue[i/2] = ((int16_t)raw[2+i] <<8) | raw[2+i+1];

      sensor_msgs::Imu corrected;

        corrected.header.stamp = ros::Time::now();
        corrected.header.frame_id = "imu_link";

        //pass calibrated acceleration to corrected IMU data object
        corrected.linear_acceleration.x = (double)intValue[0] * aRes - accelBias[0]; 
        corrected.linear_acceleration.y = (double)intValue[1] * aRes - accelBias[1]; 
        corrected.linear_acceleration.z = (double)intValue[2] * aRes - accelBias[2]; 
        
        //add calibration bias to  received angular velocity and pass to to corrected IMU data object
        corrected.angular_velocity.x = ((double)intValue[3] * gRes - gyroBias[0]) * 0.0174533; 
        corrected.angular_velocity.y = ((double)intValue[4] * gRes - gyroBias[1]) * 0.0174533; 
        corrected.angular_velocity.z = ((double)intValue[5] * gRes - gyroBias[2]) * 0.0174533; 

        // Convert gyroscope degrees/sec to radians/sec
        // gx *= 0.0174533f;
        // gy *= 0.0174533f;
        // gz *= 0.0174533f;
        //publish calibrated IMU data
        imu_raw_pub.publish(corrected);
       
      return;

      if( len < 13 )  // no mag data
        return;

        sensor_msgs::MagneticField mag_msg;
        mag_msg.header.stamp = ros::Time::now();
        mag_msg.header.frame_id = "imu_link";
        //scale received magnetic (miligauss to tesla)
        mag_msg.magnetic_field.y = ( (double) intValue[6] * mRes *magCalibration[0] - magBias[0]) * magScale[0];    //   *mRes * magCalibration[0]
        mag_msg.magnetic_field.x = ( (double) intValue[7] * mRes  *magCalibration[1] - magBias[1]) * magScale[1]; 
        mag_msg.magnetic_field.z =  -( (double) intValue[8] * mRes *magCalibration[2]  - magBias[2]) * magScale[2]; 
        
        imu_mag_pub.publish(mag_msg);
  }




void SerialMsgHandler::IMU_handle(char *buf, int len)
{
 // cout << buf;

 int ints[4];

  int ret = getIntsFromStr(ints, buf + 2, 4);
  if (ret != 4)
  {
    ROS_INFO("IM data err: %s", buf );
    return;
  }


  // broadcaster.sendTransform(
  //       tf::StampedTransform(
  //         tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.0, 0.0)),
  //         ros::Time::now(), base_link, "imu_arduino"));
	//ROS_INFO("Q: %d %d %d %d\n", ints[0], ints[1], ints[2], ints[3]);

	sensor_msgs::Imu imu_data;
            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = "imu_link"; // "imu_arduino";
//            imu_data.orientation.x = (float)ints[2]/1000; //q3
//            imu_data.orientation.y = -(float)ints[1]/1000; //-q2;
//            imu_data.orientation.z = -(float)ints[0]/1000; //-q1;
//            imu_data.orientation.w = (float)ints[3]/1000; //q4;

//1230
            imu_data.orientation.x = (float)ints[1]/1000; //q1
            imu_data.orientation.y = (float)ints[2]/1000; //q2;
            imu_data.orientation.z = (float)ints[3]/1000; //q3;
            imu_data.orientation.w = (float)ints[0]/1000; //q0;

            imu_pub.publish(imu_data);
	
}


  void SerialMsgHandler::RobotStateHandle(char *buf, int len )
  {
        double x, y, theta, w, v;
        int iVal;


        iVal = byteToInt16(buf, 2); //((data[1] & 0xff)<<8) | (data[0] & 0xff);
        x = (float) ((float)iVal/1000.0);

        iVal = byteToInt16(buf, 4); //((data[3] & 0xff)<<8) | (data[2] & 0xff);
        y = (float) ((float)iVal/1000.0);

        iVal = byteToInt16(buf, 6); //((data[5] & 0xff)<<8) | (data[4] & 0xff);
        theta = (float) ((float)iVal/1000.0);
          
        iVal = buf[8]&0xff;
        v = (float)(float)iVal/100.0;
        w = 0;
        // iVal = data[9]&0xff;
        // voltage = (float)(float)iVal/10.0;
        unsigned char pkg = (unsigned char)buf[0];
        if( pkg == 0xA1)
        {
          int leftTicks = byteToInt32(buf, 10);
          int  rightTicks = byteToInt32(buf, 14);
        }

        iVal = byteToInt16(buf, 20);
        v = (float)iVal / 1000.0;
        iVal = byteToInt16(buf, 22);
        w = (float)iVal / 1000.0;
        publishGeometryMsg(x, y, theta, w, v);

  }


void SerialMsgHandler::geometry_handle(char *buf, int len)
{
  //"RPx,y,theta,v" x=x*10000,y=y*1000,theta=theta*10000,v=%04f

  double x, y, theta, w, v;
  int ints[5];

  int ret = getIntsFromStr(ints, buf + 2, 5);
  if (ret != 5)
  {
    ROS_INFO("geo data error: %s, %d ", buf, len );
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
  t.header.frame_id =  odom;  //"odom_arduino"; //
  t.child_frame_id = base_link;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = 0.0;
  t.transform.rotation = odom_quat;
  t.header.stamp = current_time;

  if( m_publish_tf )
      broadcaster.sendTransform(t);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = odom;  //"odom_arduino";
  odom_msg.child_frame_id = base_link;

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

   odom_msg.pose.covariance[0] = 0.001;
    odom_msg.pose.covariance[7] = 0.001;
    odom_msg.pose.covariance[35] = 0.001;

  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = w;

      odom_msg.twist.covariance[0] = 0.0001;
    odom_msg.twist.covariance[7] = 0.0001;
    odom_msg.twist.covariance[35] = 0.0001;


  odom_pub.publish(odom_msg);
}

void SerialMsgHandler::IRSensor_handle(char *buf, int len)
{
  // cout << buf;
  int ints[5];

  int ret = getIntsFromStr(ints, buf + 2, 5);
  if (ret != 5)
  {
    ROS_INFO("IR data err: %s ", buf );
    //cout << "ir data error:" << ret << buf;
    return;
  }

  broadcaster.sendTransform(
      tf::StampedTransform(
          tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.0, 0.0)),
          ros::Time::now(), base_link, "sensor_frame"));

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



void SerialMsgHandler::calibDataHandle(char *buf, int len)
{
    double dins[3];
    int ret = getDoublesFromStr(dins, buf + 5, 3);
    if( ret != 3 )
    {
      ROS_INFO("Cal data err: %s", buf );
      // cout << "cm data error:" << ret << buf;
      return;
    }

    if( buf[2] == 'A' && buf[3] == 'B')
    {
      accelBias[0] = dins[0]/ 10000.0;
      accelBias[1] = dins[1]/ 10000.0;
      accelBias[2] = dins[2]/ 10000.0;
      return;
    }

    double *p = NULL;
     if( buf[2] == 'G' && buf[3] == 'B')
      p = gyroBias;
    else if( buf[2] == 'M' && buf[3] == 'B')
      p = magBias;
    else if( buf[2] == 'M' && buf[3] == 'S')
      p = magScale;
    else if( buf[2] == 'M' && buf[3] == 'C')
      p = magCalibration;

    if( p == NULL )
      return;
    for( int i=0; i<3; i++ )
    {
      *(p+i) = dins[i];
    }
}

void SerialMsgHandler::IMU_RawDataHandle(char *buf, int len)
{
    int ints[9];

  int ret = getIntsFromStr(ints, buf + 2, 9);
  if (ret != 9)
  {
    ROS_INFO("IM raw data err: %s", buf );
    return;
  }


  // broadcaster.sendTransform(
  //     tf::StampedTransform(
  //         tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.0, 0.0)),
  //         ros::Time::now(), "base_link", "imu_link"));

sensor_msgs::Imu corrected;

  corrected.header.stamp = ros::Time::now();
  corrected.header.frame_id = "imu_link";

  //pass calibrated acceleration to corrected IMU data object
  corrected.linear_acceleration.x = (double)ints[0] * aRes - accelBias[0]; 
  corrected.linear_acceleration.y = (double)ints[1] * aRes - accelBias[1]; 
  corrected.linear_acceleration.z = (double)ints[2] * aRes - accelBias[2]; 
  
  //add calibration bias to  received angular velocity and pass to to corrected IMU data object
  corrected.angular_velocity.x = ((double)ints[3] * gRes - gyroBias[0]) * 0.0174533; 
  corrected.angular_velocity.y = ((double)ints[4] * gRes - gyroBias[1]) * 0.0174533; 
  corrected.angular_velocity.z = ((double)ints[5] * gRes - gyroBias[2]) * 0.0174533; 

	// Convert gyroscope degrees/sec to radians/sec
	// gx *= 0.0174533f;
	// gy *= 0.0174533f;
	// gz *= 0.0174533f;


  //publish calibrated IMU data
  imu_raw_pub.publish(corrected);


  sensor_msgs::MagneticField mag_msg;

  mag_msg.header.stamp = ros::Time::now();
  mag_msg.header.frame_id = "imu_link";
  //scale received magnetic (miligauss to tesla)
  mag_msg.magnetic_field.y = ( (double) ints[6] * mRes *magCalibration[0] - magBias[0]) * magScale[0];    //   *mRes * magCalibration[0]
  mag_msg.magnetic_field.x = ( (double) ints[7] * mRes  *magCalibration[1] - magBias[1]) * magScale[1]; 
  mag_msg.magnetic_field.z =  -( (double) ints[8] * mRes *magCalibration[2]  - magBias[2]) * magScale[2]; 
  
  imu_mag_pub.publish(mag_msg);

}


void serialMsgThread(SerialMsgHandler *pHandler, long timeout)
{
  pHandler->serialMsgLoop(timeout);
}

/**
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
          if (idx > 2 && buf[0] == 'R' && buf[1] == 'P')  //robot position
          {
            if (geomProc != NULL)
              geomProc(buf, idx);
            else
              cout << buf;
          }
          else if (idx > 2 && buf[0] == 'I' && buf[1] == 'R') //ir sensor info
          {
            IRSensorProc(buf, idx);
          }
          else if (idx > 2 && buf[0] == 'I' && buf[1] == 'M')  // filtered imu 四元素
          {
            IMUProc(buf, idx);
          }
          else if( idx >2 && buf[0] == 'R' && buf[1] == 'E')  //ready
          {
            cout << "robot ready!" << endl;

              cout << "send connect cmd CR..." << endl;
              serial->write( "\ncr;\n" , 4);
          }

          else
          {
            cout <<  idx << buf << endl;
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
*/

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

  ROS_INFO("twist cmd:%s", cmd  );
  
  
}


bool mSaveCalibration = false;

bool execCmd(zmcrobot_ros::ExecCmd::Request &req, 
          zmcrobot_ros::ExecCmd::Response &res )
          {
            // cout << "exec cmd: " << req.cmd << endl;
           ROS_INFO("Exec CMD:%s", req.cmd.c_str());
            res.retStr = "OK";
            if( req.cmd == "sc;" || req.cmd == "sc" )
              mSaveCalibration = true;

            if (m_pSerialPort != NULL)
              m_pSerialPort->write((const void *)(char *)req.cmd.c_str(), req.cmd.length());
            return true;
          }


int main(int argc, char **argv)
{

  ros::init(argc, argv, "base_controller");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("cmd_vel", 50, handle_twist);

  ros::ServiceServer service = nh.advertiseService("exec_cmd", execCmd);
//本节点空间
// 波浪符代表节点句柄的命名空间，其作用与C++的”this”指针和python中的”self”类似.在ros中可以使用 roslaunch 进行参数传递. 在 launch 标签内的是全局参数，而在node 标签内的则是局部参数。如果需要取全局的,需要名字前面加'/'; 如果要用 nh 取参数,则需要在名字前面加 node name : "nodename/paramnane"
  ros::NodeHandle nlh("~");
  int baud = 115200;
  string port = "/dev/ttyACM0";
  bool simulateMode = true;
  int timeout = 500000;
  bool use_imu = false;
  float alpha = 0.8;

  bool publish_tf = false; // default to false, publish the odom to baselink transfer if true
  string calib_file;
  bool record_data  = false;


  nlh.param("baud", baud, baud);
  nlh.param("port", port, port);
  nlh.param("simulate", simulateMode, simulateMode);
  nlh.param("timeout", timeout, timeout);

  nlh.param("publish_tf", publish_tf, publish_tf);  
  nlh.param("record_data", record_data, record_data);

  nlh.param("use_imu", use_imu, use_imu);
  nlh.param("alpha", alpha, alpha);
  nlh.param("calib_file", calib_file, calib_file);

  // if( argc>2 )
  //     baudrate = atoi( argv[2]);

	// cout << "value:" << port << endl;
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

  SerialMsgHandler msgHandler(&serialPort);

  msgHandler.loadCalibrateParams(calib_file);

  if( publish_tf )
      ROS_INFO("publish odom to base link transfer!\n");
  
  if( record_data )
  {
    ROS_INFO("Record the serial data.\n" );
    msgHandler.setRecordData( true );
  }

  msgHandler.m_publish_tf = publish_tf;

  //start the serial msg read and handle thread
  std::thread msgHandleThread(serialMsgThread, &msgHandler, timeout);

//   if (simulateMode == true)
//   {
//     cout << "set to simulate mode ...\n";
//     serialPort.write("\nsm1\n", 5);
//   }

//  if( use_imu )
//  {
//      ROS_INFO("set use imu with alpha: %.2f\n", alpha);
//      serialPort.printf("im1,%.2f\n", alpha);
//  }

  // int idx;
  // char buf[200];


  tf::TransformBroadcaster broadcaster;
  double rate = 10.0;
  ros::Rate r(rate);
  ros::Time current_time;

  // signal(SIGINT,quit);

  while (nh.ok())
  {
    ros::spinOnce();
    // ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("rpm", n, d);
    current_time = ros::Time::now();

    if( mSaveCalibration )
    {
      ROS_INFO("do save calibration ... ");
      msgHandler.saveCalibration();
      mSaveCalibration = false;
    }
    r.sleep();
  }
  cout << "Be quit... " << endl;
  msgHandler.close();
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



int getDoublesFromStr(double *dins, const char *buf, int count)
{
  if (dins == NULL || buf == NULL)
    return 0;
  int cnt = 0;
  const char *p = buf;
  while (cnt < count)
  {
    dins[cnt++] = atof(p);
    p = strchr(p, ',');
    if (p == NULL)
      return cnt;
    p++;
  }
  return cnt;
}
