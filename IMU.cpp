#include "IMU.h"

/*************************
 * ****************************************************-
 *                                   IMU.cpp
 *****************************************************************************/

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
    imuInit()
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
  ICM_20948_dlpcfg_t myDLPcfg;            // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d111bw4_n136bw;         // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                          // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                          // acc_d111bw4_n136bw
                                          // acc_d50bw4_n68bw8
                                          // acc_d23bw9_n34bw4
                                          // acc_d11bw5_n17bw
                                          // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                          // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d119bw5_n154bw3;       // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                          // gyr_d196bw6_n229bw8
                                          // gyr_d151bw8_n187bw6
                                          // gyr_d119bw5_n154bw3
                                          // gyr_d51bw2_n73bw3
                                          // gyr_d23bw9_n35bw9
                                          // gyr_d11bw6_n17bw8
                                          // gyr_d5bw7_n8bw9
                                          // gyr_d361bw4_n376bw5

  imu.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
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
  static float gYaw = 0.0;

  gyroPitchDelta = -gyroY / sampleFreq; // degrees changed during period
  gPitch += gyroPitchDelta;   // Debugging
  gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle

  float gyroRollDelta = gyroX / sampleFreq;
  gRoll += gyroRollDelta;
  gaRoll = gaRoll - gyroRollDelta;

  float gyroYawDelta = -gyroZ / sampleFreq; // degrees changed during period
  gYaw += gyroYawDelta;

  float aPitch = ((atan2(-accelY, accelZ)) * RAD_TO_DEG);
  gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));

  // Roll
  float aRoll =  (atan2(accelY, accelZ) * RAD_TO_DEG);
  gaRoll = (gaRoll * GYRO_WEIGHT) + (aRoll * (1 - GYRO_WEIGHT)); // Weigh factors
}


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/*****************************************************************************-
     MahonyAHRSupdateIMU()

     From: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

 *****************************************************************************/


#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

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
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
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
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
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
//    static unsigned long t1 = 0UL;
//    unsigned long t2 = micros();
    imu.getAGMT();
//    printScaledAGMT();   // This function takes into account the sclae settings from when the measurement was made to calculate the values with units

    float accelX   = imu.accX() / 1000;  // divide to get units in g
    float accelY   = imu.accY() / 1000;  // divide to get units in g
    float accelZ   = imu.accZ() / 1000;  // divide to get units in g

    float gyroPitchDelta = imu.gyrX();
    float gyroXrad = gyroPitchDelta * DEG_TO_RAD;                  // radians/sec
    float gyroRollDelta = imu.gyrY();
    float gyroYrad = gyroRollDelta * DEG_TO_RAD;                  // radians/sec
    float gyroYawDelta = imu.gyrZ();
    float gyroZrad = gyroYawDelta * DEG_TO_RAD;                  // radians/sec

//    sprintf(message, "AG: %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f", accelX, accelY, accelZ, gyroPitchDelta, gyroRollDelta, gyroYawDelta);
//    _serialOut->println(message);

    compFilter(gyroPitchDelta, gyroRollDelta, gyroYawDelta, accelX, accelY, accelZ);
    accelUpdate();
    //    MadgwickQuaternionUpdate(accelX, accelY, accelZ, gyroXrad, gyroYrad, gyroZrad);
    MahonyAHRSupdateIMU(gyroXrad, gyroYrad, gyroZrad, accelX, accelY, accelZ);

    maRollRad  = -atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    maPitchRad = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    maYawRad   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    maPitch = maPitchRad * RAD_TO_DEG;
    maRoll  = maRollRad * RAD_TO_DEG;
    maYaw   = maYawRad * RAD_TO_DEG;


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
//  tireZAccel = (cos(gaPitch * DEG_TO_RAD) * accelZ) + (sin(gaPitch * DEG_TO_RAD) * accelY);
//  tireYAccel = (sin(gaPitch * DEG_TO_RAD) * accelZ) + (cos(gaPitch * DEG_TO_RAD) * accelY);
}

void IMU::printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    _serialOut->print("-");
  }else{
    _serialOut->print(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      _serialOut->print("0");
    }else{
      break;
    }
  }
  if(val < 0){
    _serialOut->print(-val, decimals);
  }else{
    _serialOut->print(val, decimals);
  }
}

void IMU::printScaledAGMT(){
  _serialOut->print("Scaled. Acc (mg) [ ");
  printFormattedFloat( imu.accX(), 5, 2 );
  _serialOut->print(", ");
  printFormattedFloat( imu.accY(), 5, 2 );
  _serialOut->print(", ");
  printFormattedFloat( imu.accZ(), 5, 2 );
  _serialOut->print(" ], Gyr (DPS) [ ");
  printFormattedFloat( imu.gyrX(), 5, 2 );
  _serialOut->print(", ");
  printFormattedFloat( imu.gyrY(), 5, 2 );
  _serialOut->print(", ");
  printFormattedFloat( imu.gyrZ(), 5, 2 );
  _serialOut->print(" ], Mag (uT) [ ");
  printFormattedFloat( imu.magX(), 5, 2 );
  _serialOut->print(", ");
  printFormattedFloat( imu.magY(), 5, 2 );
  _serialOut->print(", ");
  printFormattedFloat( imu.magZ(), 5, 2 );
  _serialOut->print(" ], Tmp (C) [ ");
  printFormattedFloat( imu.temp(), 5, 2 );
  _serialOut->print(" ]");
  _serialOut->println();
}
