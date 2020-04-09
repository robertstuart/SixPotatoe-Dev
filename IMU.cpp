/*****************************************************************************-
 *                                 IMU.cpp
 *****************************************************************************/
#include "IMU.h"
#define sampleFreq  200.0f      // sample frequency in Hz
#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address.
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when
                        // the ADR jumper is closed the value becomes 0

float zeta = 0;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.05f;                              // integration interval for both filter schemes
float q[4] = {0.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion

// Base
IMU::IMU() {
  // Default to using Serial for output
  _serialOut = &Serial;
}

/*****************************************************************************-
 *   imuInit()
 *****************************************************************************/
void IMU::imuInit() {
  imuInit(&Serial, WIRE_PORT, AD0_VAL);
}

void IMU::imuInit(Stream *serialImpl) {
  imuInit(serialImpl, WIRE_PORT, AD0_VAL);
}

void IMU::imuInit(Stream *serialImpl, TwoWire &wirePort, uint8_t ad0_val) {
  _serialOut = serialImpl;

  bool initialized = false;
  while( !initialized ) {
    // start communication with IMU
    imu.begin( wirePort, ad0_val );
    _serialOut->print( F("Initialization of the sensor returned: ") );
    _serialOut->println( imu.statusString() );
    if( imu.status != ICM_20948_Stat_Ok ){
      _serialOut->println( "Trying again..." );
      delay(500);
    }else{
      initialized = true;
    }
  }

  // Set sample rate
  setSampleRate(sampleFreq);

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm8;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                          // gpm2
                          // gpm4
                          // gpm8
                          // gpm16

  myFSS.g = dps2000;      // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                          // dps250
                          // dps500
                          // dps1000
                          // dps2000

  imu.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );
  if( imu.status != ICM_20948_Stat_Ok){
    _serialOut->print(F("setFullScale returned: "));
    _serialOut->println(imu.statusString());
  }

  // Set up Digital Low-Pass Filter configuration
  // Similar to FSS, this uses a configuration structure for the desired sensors
  
  // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
  // Format is dAbwB_nXbwZ - A is integer part of 3db BW, B is fraction. X is integer part of nyquist bandwidth, Y is fraction
  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
  // acc_d111bw4_n136bw
  // acc_d50bw4_n68bw8
  // acc_d23bw9_n34bw4
  // acc_d11bw5_n17bw
  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
  // acc_d473bw_n499bw

  // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
  // Format is dAbwB_nXbwZ - A is integer part of 3db BW, B is fraction. X is integer part of nyquist bandwidth, Y is fraction
  // gyr_d196bw6_n229bw8
  // gyr_d151bw8_n187bw6
  // gyr_d119bw5_n154bw3
  // gyr_d51bw2_n73bw3
  // gyr_d23bw9_n35bw9
  // gyr_d11bw6_n17bw8
  // gyr_d5bw7_n8bw9
  // gyr_d361bw4_n376bw5
  
  ICM_20948_dlpcfg_t myDLPcfg; 
  myDLPcfg.a = acc_d23bw9_n34bw4; 
  myDLPcfg.g = gyr_d119bw5_n154bw3;                                    

  imu.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
  imu.enableDLPF( ICM_20948_Internal_Acc, true );
  imu.enableDLPF( ICM_20948_Internal_Gyr, true );

  if( imu.status != ICM_20948_Stat_Ok){
    _serialOut->print(F("setDLPcfg returned: "));
    _serialOut->println(imu.statusString());
  }


  _serialOut->println();
  _serialOut->println(F("Configuration complete!"));

}

/*******************************************************************-
 * The documentation for the ICM-20948 states that the
 * GYRO_SMPLRT_DIV and ACCEL_SMPLRT_DIV(1/2) registers
 * "Divide the internal sample rate to generate the sample rate
 *  that controls sensor data output rate, FIFO sample rate, and
 *  DMP sequence rate."
 *  The Data Output Rate is computed as: 1.125 / (1 + GYRO_SMPLRT_DIV).
 */
void IMU::setSampleRate(float smplFreq) {
    ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = (1125/smplFreq) -1;
  mySmplrt.a = (1125/smplFreq) -1;
  imu.setSampleRate( ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, mySmplrt );
  _serialOut->print(F("setSampleRate returned: "));
  _serialOut->println(imu.statusString());
}



/*****************************************************************************-
 *  compFilter()  Complementary filter to get angles
 *****************************************************************************/
void IMU::compFilter(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ) {
  static float gPitch = 0.0;
  static float gRoll = 0.0;

  // Pitch
  gyroPitchDelta = -gyroX / sampleFreq; // degrees changed during period
  gPitch += gyroPitchDelta;   // Debugging
  gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
  float aPitch = ((atan2(-accelY, accelZ)) * RAD_TO_DEG);
  gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));

  // Roll
  float gyroRollDelta = gyroY / sampleFreq;
  gRoll += gyroRollDelta;
  gaRoll = gaRoll - gyroRollDelta;
  float aRoll =  (atan2(accelX, accelZ) * RAD_TO_DEG);
  gaRoll = (gaRoll * GYRO_WEIGHT) + (aRoll * (1 - GYRO_WEIGHT)); // Weigh factors

  // Yaw/Heading
  float gyroYawDelta = -gyroZ / sampleFreq; // degrees changed during period
  gHeading += gyroYawDelta;
}



/*****************************************************************************-
     MahonyAHRSupdateIMU()

     From: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

 *****************************************************************************/
#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

volatile float twoKp = twoKpDef;                      // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;                      // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q[0] = q0 *= recipNorm;
  q[1] = q1 *= recipNorm;
  q[2] = q2 *= recipNorm;
  q[3] = q3 *= recipNorm;
} // End MahonyAHRSupdateIMU()

/*****************************************************************************-
    isNewImuData()   Returns true if the IMU has new data.
                Reads the IMU and sets the new values.
 *****************************************************************************/
boolean IMU::isNewImuData() {

  if (imu.dataReady()) {
    imu.getAGMT();
    accelX   = imu.accX() / 1000;  // divide to get units in g
    accelY   = imu.accY() / 1000;  // divide to get units in g
    accelZ   = imu.accZ() / 1000;  // divide to get units in g

    float gyroPitchDelta = imu.gyrX() + 1.4; // better to do offset dynamically TODO
    float gyroXrad = gyroPitchDelta * DEG_TO_RAD;                  // radians/sec
    float gyroRollDelta = imu.gyrY();
    float gyroYrad = gyroRollDelta * DEG_TO_RAD;                  // radians/sec
    float gyroYawDelta = imu.gyrZ();
    float gyroZrad = gyroYawDelta * DEG_TO_RAD;                  // radians/sec

//    sprintf(message, "AG: %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f", accelX, accelY, accelZ, gyroPitchDelta, gyroRollDelta, gyroYawDelta);
//    _serialOut->println(message);

    //    MadgwickQuaternionUpdate(accelX, accelY, accelZ, gyroXrad, gyroYrad, gyroZrad);
    MahonyAHRSupdateIMU(gyroXrad, gyroYrad, gyroZrad, accelX, accelY, accelZ);

    maPitchRad  = -atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    maRollRad = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    maYawRad   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    maPitch = maPitchRad * RAD_TO_DEG;
    maRoll  = maRollRad * RAD_TO_DEG;
    maYaw   = maYawRad * RAD_TO_DEG;

    compFilter(gyroPitchDelta, gyroRollDelta, gyroYawDelta, accelX, accelY, accelZ);
    accelUpdate();

//    sprintf(message, "%7.2f %7.2f %7.2f %7.2f %7.2f %7d", maPitch, maRoll, maYaw, gaPitch, gaRoll, t2 - t1);
//    _serialOut->println(message);
//    t1 = t2;

    return true;
  } else {
    return false; // no IMU read
  }
}



/*****************************************************************************-
 *  accelUpdate()  Compute vertical acceleration and horizontal speed from
 *                 the acceleration along the y/x? axis.
 *****************************************************************************/
void IMU::accelUpdate() {
}
