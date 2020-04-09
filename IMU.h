#ifndef _IMU_H_
#define _IMU_H_

#include "ICM_20948.h"

const float GYRO_WEIGHT = 0.997;

// Base
class IMU
{
private:
  Stream *_serialOut;
  ICM_20948_I2C imu;
  char message[80];

  void accelSet(float accelX, float accelY, float accelZ);
  void accelUpdate();
  void compFilter(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ);
  void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz);
  void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);
  void setSampleRate(float smplFreq);

  float maPitchRad = 0.0;
  float maRollRad = 0.0;
  float maYawRad = 0.0;


protected:

public:
  IMU(); // Constructor

  void imuInit();
  void imuInit(Stream *serialImpl);
  void imuInit(Stream *serialImpl, TwoWire &wirePort, uint8_t ad0_val);
  boolean isNewImuData();
  void printScaledAGMT();
  
  float maPitch = 0;
  float maRoll = 0.0;
  float maYaw = 0.0;
  
  float gyroPitchDelta = 0;
  float gaPitch = 0;
  float gHeading = 0;
  float gaRoll = 0;
  float accelX = 0.0;  
  float accelY = 0.0; 
  float accelZ = 0.0;

  float vertAccel = 0.0;
  float horizAccel = 0.0;
  float horizSpeed = 0.0;

};

#endif /* _IMU_H_ */
