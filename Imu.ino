/*****************************************************************************-
 *                                   IMU.ino
 *****************************************************************************/
//#define MPU9250_ADDRESS   MPU9250_ADDRESS_AD0 // 0x68
#define sampleFreq  200.0f      // sample frequency in Hz
const float GYRO_SENS = 0.06097;      // Multiplier to get degrees.
const float ACCEL_SENSE = 1.0 / 4098.0;       // Multiplier to get force in g's.

//LSM6 lsm6;
//MPU9250 imu(MPU9250_ADDRESS, Wire, 400000);
MPU9250_DMP imu;

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
int zSum = 0;
int zCount = 0;
int yawTempComp = 0;


// parameters for 6 DoF sensor fusion calculations
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
//float GyroMeasError = PI * (5.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
//float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float zeta = 0;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.05f;                              // integration interval for both filter schemes
float q[4] = {0.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion

/*****************************************************************************-
    imuInit()
 *****************************************************************************/
void imuInit() {
  if (imu.begin() != INV_SUCCESS) {
    while (1) {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println();
      delay(5000);
    }
  }

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  imu.setSampleRate(200); // Set accel/gyro sample rate to 4Hz
  imu.setAccelFSR(8);
  imu.setGyroFSR(2000);
  imu.setLPF(98);
  imu.enableInterrupt();
  imu.setIntLevel(INT_ACTIVE_HIGH);
  imu.setIntLatched(INT_LATCHED);

  // Clear out
  delay(10);
  imu.update(UPDATE_ACCEL | UPDATE_GYRO);
  delay(10);
  imu.update(UPDATE_ACCEL | UPDATE_GYRO);
}


#define TG_PITCH_TC 0.90D



/*****************************************************************************-
    isNewImu()   Returns true if the IMU has new data.
                Reads the IMU and sets the new values.
 *****************************************************************************/
boolean isNewImuData() {
  static int oldT = 0;

  if (imu.dataReady()) {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO);

    accelX = ((float) imu.ax) * ACCEL_SENSE;
    accelY = ((float) imu.ay) * ACCEL_SENSE;
    accelZ = ((float) imu.az) * ACCEL_SENSE;

    float gyroPitchRaw = (float) imu.gx;                  // drift compenstation
    float gyroPitchDelta  = gyroPitchRaw * GYRO_SENS;    // degrees/sec
    float gyroXrad = gyroPitchDelta * DEG_TO_RAD;                  // radians/sec

    float gyroRollRaw = (float) imu.gy;
    float gyroRollDelta  = gyroRollRaw * GYRO_SENS;              // degrees/sec
    float gyroYrad = gyroRollDelta * DEG_TO_RAD;                  // radians/sec

    float gyroYawRaw = (float) imu.gz;
    float gyroYawDelta  = gyroYawRaw * GYRO_SENS;                // degrees/sec
    float gyroZrad = gyroYawDelta * DEG_TO_RAD;                   // radians/sec

    compFilter(gyroPitchDelta, gyroRollDelta, gyroYawDelta, accelX, accelY, accelZ);
    MahonyAHRSupdateIMU(gyroXrad, gyroYrad, gyroZrad, accelX, accelY, accelZ);

    maPitch  = -atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    float maRoll = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    float maYaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    maPitch *= RAD_TO_DEG;
    maRoll  *= RAD_TO_DEG;
    maYaw   *= RAD_TO_DEG;

    sprintf(message, "%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f", maPitch, maRoll, maYaw, gaPitch, gaRoll, gYaw);
    Serial.println(message);

    return true;
  } else {
    return false; // no IMU read
  }
}



/*****************************************************************************-
 *  compFilter()  Complementary filter to get angles
 *****************************************************************************/
void compFilter(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ) {
  static float gPitch = 0.0;
  static float gRoll = 0.0;
  static float gYaw = 0.0;
  
  gyroPitchDelta = -gyroX / sampleFreq; // degrees changed during period
  gPitch += gyroPitchDelta;   // Debugging
  gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle

  float gyroRollDelta = gyroY / sampleFreq;
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
