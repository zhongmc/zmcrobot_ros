#include "IRSensor.h"
#include <math.h>

//fallow wall 时给出的无障碍物时的坐标距离
#define WALL_DIS 0.35

IRSensor::IRSensor()
{
  mSensorType = GP2Y0A21;
  x_s = 0;
  y_s = 0;
  theta_s = 0;

  init();
}

IRSensor::IRSensor(SENSOR_TYPE sensorType)
{
  x_s = 0;
  y_s = 0;
  theta_s = 0;
  mSensorType = sensorType;
  init();
}

IRSensor::IRSensor(double xs, double ys, double thetas, SENSOR_TYPE sensorType)
{
  x_s = xs;
  y_s = ys;
  theta_s = thetas;
  mSensorType = sensorType;
  init();
}

void IRSensor::init()
{
  setDistance(getMaxDistance());
  lastDistance = distance;
}

void IRSensor::SetSensorType(SENSOR_TYPE sensorType)
{
  mSensorType = sensorType;
  setDistance(getMaxDistance());
}

void IRSensor::setDistance(double dis)
{
  distance = dis;
  x = x_s + distance * cos(theta_s);
  y = y_s + distance * sin(theta_s);
}

//由机器人坐标，转换到物理坐标
void IRSensor::applyGeometry(double xc, double yc, double sinTheta, double cosTheta)
{
  xw = xc + x * cosTheta - y * sinTheta;
  yw = yc + x * sinTheta + y * cosTheta;
}

double IRSensor::getMaxDistance()
{
  if (mSensorType == GP2Y0A21)
    return 0.8;
  else if (mSensorType == GP2Y0A41)
    return 0.3;
  return 1;
}
double IRSensor::getMinDistance()
{
  if (mSensorType == GP2Y0A21)
    return 0.1;
  else if (mSensorType == GP2Y0A41)
    return 0.04;
  return 0.1;
}

// double IRSensor::getDistance(int digitalVal)
// {
//   double d;
//   if (mSensorType == GP2Y0A21) //1/d = 0.0148x - 0.8085
//   {
//     if (digitalVal > 760)
//       return 0.01;
//     if (digitalVal < 120)
//       return 0.8;
//     d = 1.0 / (0.0148 * digitalVal - 0.8085);
//     return d;
//   }
//   else if (mSensorType == GP2Y0A41) //  y = 0.0293x - 0.9342
//   {
//     if (digitalVal > 860)
//       return 0.04;
//     if (digitalVal < 105)
//       return 0.3;

//     d = 1.0 / (0.0293 * digitalVal - 0.9342);
//     return d;
//   }
//   else
//   {
//     return 0.8;
//   }
// }
