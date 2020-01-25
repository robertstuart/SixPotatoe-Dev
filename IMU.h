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
  void compFilter(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ);
  void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz);
  void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);
  void setSampleRate(float smplFreq);

protected:

public:
    IMU(); // Constructor

    void imuInit();
    void imuInit(Stream *serialImpl);
    void imuInit(Stream *serialImpl, TwoWire &wirePort, uint8_t ad0_val);
    boolean isNewImuData();
    void printScaledAGMT();
    float maPitch = 0;
    float gyroPitchDelta = 0;
    float gaPitch = 0;
    float gYaw = 0;
    float gaRoll = 0;
};

#endif /* _IMU_H_ */
