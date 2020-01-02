/******************************************************************************
 *  Imu.ino
 *****************************************************************************/
const int zMax = 1148;

int zSum = 0;
int zCount = 0;
int yawTempComp = 0;
/******************************************************************************
 *  angleInit()
 *****************************************************************************/
void imuInit() {
  int success;
  Wire.begin();
  Wire.setClock(400000);
  success = lsm6.init(); 
  if (lsm6.init()) Serial.println("IMU Initialized!");
  else Serial.println("IMU initialize failed!"); 
  lsm6.enableDefault();
  lsm6.writeReg(LSM6::INT1_CTRL, 0X02); // Accel data ready on INT1
  lsm6.writeReg(LSM6::INT2_CTRL, 0X01); // Gyro data ready on INT2
  lsm6.writeReg(LSM6::CTRL2_G, 0X5C);   // Gyro 2000fs, 208hz
  lsm6.writeReg(LSM6::CTRL1_XL, 0X50);  // Accel 2g, 208hz
}



/******************************************************************************
 * isNewImuData()  Return true if new data has been read.
 *   TODO change this to use poll of IMU
 **************************************************************************/
boolean isNewImuData() {
  static unsigned long imuTrigger = 0UL;
  if (timeMicroseconds > imuTrigger) {
    imuTrigger = timeMicroseconds + 5000;
    lsm6.readGyro();
    lsm6.readAcc();
    
    float gyroPitchRaw = ((float) lsm6.g.x);
    float gyroPitchRate = (((float) gyroPitchRaw) * GYRO_SENS);  // Rate in degreesChange/sec
    gyroPitchDelta = -gyroPitchRate / 208.0; // degrees changed during period
    float gPitch = gPitch + gyroPitchDelta;   // Used by tgPitch & debugging
    gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
    float accelX = lsm6.a.y;  // 
    float aPitch = ((atan2(accelX, -lsm6.a.z)) * RAD_TO_DEG) + ACCEL_PITCH_OFFSET;
    gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
    return true;
  }
  return false;
}
