#ifndef _IRSENSOR_H_
#define _IRSENSOR_H_

typedef enum
{
  GP2Y0A41 = 0, //4-30cm
  GP2Y0A21 = 1  //10-80cm
} SENSOR_TYPE;

class IRSensor
{
public:
  IRSensor();
  IRSensor(SENSOR_TYPE sensorType);

  IRSensor(double xs, double ys, double thetas, SENSOR_TYPE sensorType);
  void readPosition();
  void readPosition(double xs, double ys, double thetas);
  void setDistance(double val);

  double distance, lastDistance; //obstacle distance
  double x, y;                   //the obtacle position in robot geometry
  double xw, yw;                 //the obtacle pos in real geometry

  void SetSensorType(SENSOR_TYPE sensorType);

  SENSOR_TYPE getSensorType()
  {
    return mSensorType;
  };

  void applyGeometry(double xc, double yc, double sinTheta, double cosTheta);
  double getDistance(int digitalVal);
  double getMaxDistance();
  double getMinDistance();

private:
  void init();
  double x_s, y_s, theta_s; //sensor position
  SENSOR_TYPE mSensorType;
};

#endif /* _IRSENSOR_H_ */
