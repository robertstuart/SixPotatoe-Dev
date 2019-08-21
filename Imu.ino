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
  for (int i = 0; i < 100; i++) {
    success = lsm6.init(); 
    if (success) break; 
    Serial.println("IMU initialize failed!");
    resetIMU();
    Wire.begin();
  }
  if (success) Serial.println("IMU Initialized!****************************");
    lsm6Init();
}

void lsm6Init() {
  lsm6.enableDefault();
  lsm6.writeReg(LSM6::INT1_CTRL, 0X02); // Accel data ready on INT1
  lsm6.writeReg(LSM6::INT2_CTRL, 0X01); // Gyro data ready on INT2
  lsm6.writeReg(LSM6::CTRL2_G, 0X5C);   // Gyro 2000fs, 208hz
  lsm6.writeReg(LSM6::CTRL1_XL, 0X50);  // Accel 2g, 208hz
}

#define TG_PITCH_TC 0.90D

/******************************************************************************
 * setGyroData()
 *****************************************************************************/
void setGyroData() {
  
  // Pitch
  gyroPitchRaw = ((float) lsm6.g.x) - timeDriftPitch;
  gyroPitchRate = (((float) gyroPitchRaw) * GYRO_SENS);  // Rate in degreesChange/sec
  gyroPitchDelta = -gyroPitchRate / 208.0; // degrees changed during period
  gPitch = gPitch + gyroPitchDelta;   // Used by tgPitch & debugging
  gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
//  gaFullPitch = gyroPitchDelta + gaFullPitch;
  
  // Roll
  gyroRollRaw = ((float) lsm6.g.y) - timeDriftRoll;
  gyroRollRate = (((float) gyroRollRaw) * GYRO_SENS);
  float gyroRollDelta = gyroRollRate / 208.0;
  gRoll = gRoll - gyroRollDelta;
  gaRoll = gaRoll - gyroRollDelta;

  // Yaw
  gyroYawRaw = ((double) lsm6.g.z) - timeDriftYaw; 
  gyroYawRate = ((double) gyroYawRaw) * GYRO_SENS;  // Rate in degreesChange/sec
  double gyroYawDelta = -gyroYawRate / 208.0; // degrees changed during period
  gYaw += gyroYawDelta;
  gHeading = rangeAngle(gYaw); 

  // Tilt compensated heading
  double yawDeltaError = gyroYawDelta * gaPitch * gaPitch * 0.000135;
  gcYaw += gyroYawDelta + yawDeltaError; 
  gcHeading = rangeAngle(gcYaw);  // This is the heading used by all navigation routines.
}


static double APITCH_TC = 0.99;
/******************************************************************************
 *  setAccelData()
 *****************************************************************************/
void setAccelData() {
  static int lastAccel;
  double oldLpfAPitch = 0;
  // Pitch
//  double accelX = lsm6.a.y + (CONST_ACCEL_PITCH * 100000.0 * lpfCos3Accel);  // 
  double accelX = lsm6.a.y;  // 
  aPitch = ((atan2(accelX, -lsm6.a.z)) * RAD_TO_DEG) + ACCEL_PITCH_OFFSET;
//  gaFullPitch = (gaFullPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
//  if ((          (lsm6.a.z > 7000)
//             && ((accelX > -7000) && (accelX < 7000))
//             && ((aPitch > -45.0) && (aPitch < 45.0)))/* || !isRunning */ ) {
//      gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
//    }
  gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
  lpfAPitch = (oldLpfAPitch * APITCH_TC) + (aPitch * (1.0 - APITCH_TC));
  oldLpfAPitch = lpfAPitch;

  // y accel
  yAccel += lastAccel - lsm6.a.y;
  lastAccel = lsm6.a.y;
  
  // Roll
  aRoll =  (atan2(lsm6.a.x, -lsm6.a.z) * RAD_TO_DEG);
  gaRoll = (gaRoll * GYRO_WEIGHT) + (aRoll * (1 - GYRO_WEIGHT)); // Weigh factors
}


//#define GM_HEADING_TC 0.95D
#define GM_HEADING_TC 0.98D
#define TM_HEADING_TC 0.999D


 
/******************************************************************************
 *  setNavigation() Set gmHeading, tmHeading, tickHeading, currentLoc
 *                  Called 208/sec (every read of gyro).
 *****************************************************************************/
#define TICK_BIAS_TRIGGER 500
void setNavigation() {
  static int navOldTickPosition = 0;

  tickPosition = tickPositionRight + tickPositionLeft;

 // compute the Center of Oscillation Tick Position
  coTickPosition = tickPosition - ((long) (sin(gaPitch * DEG_TO_RAD) * 4000.0));

  // Compute the new co position
  double dist = ((double) (coTickPosition - navOldTickPosition)) / TICKS_PER_FOOT;
  navOldTickPosition = coTickPosition;
  currentLoc.x += sin(gHeading * DEG_TO_RAD) * dist;
  currentLoc.y += cos(gHeading * DEG_TO_RAD) * dist;

  currentAccelLoc();
}




//double accelFpsSelfX = 0.0;
//double accelFpsSelfY = 0.0;
//double accelFpsMapX = 0.0;
//double accelFpsMapY = 0.0;
//struct loc currentAccelSelfLoc;
//struct loc currentAccelMapLoc;

const double A_FACTOR = .000001D;
/******************************************************************************
 *  setAccelLoc() Set currentAccelLoc
 *****************************************************************************/
void currentAccelLoc() {
//  currentAccelMapLoc.y += ((double) compass.a.y) * A_FACTOR;
//  currentAccelMapLoc.x += ((double) compass.a.x) * A_FACTOR;
  //  currentAccelMapLoc.x += accelFpsSelfX * .0025;
  //  currentAccelMapLoc.y += accelFpsSelfY * .0025;
  //  accelFpsMapX = (sin(currentMapHeading * DEG_TO_RAD) * accelFpsSelfX) + (cos(currentMapHeading * DEG_TO_RAD) * accelFpsSelfY);
  //  accelFpsMapY = (cos(currentMapHeading * DEG_TO_RAD) * accelFpsSelfX) + (sin(currentMapHeading * DEG_TO_RAD) * accelFpsSelfY);
  //  currentAccelMapLoc.x += accelFpsMapX * 0.0025;
  //  currentAccelMapLoc.y += accelFpsMapY * 0.0025;
}



/******************************************************************************
 * setHeading() Sets the bearing to the new value.  The the gridOffset
 *              value will be set so that the gridBearing is an
 *              offset from magHeading.  All of cumulative rotations 
 *              be lost.
 **************************************************************************/
void setHeading(double newHeading) {
  gcHeading = gHeading = gcYaw = gYaw = newHeading;
}

void resetTicks() {
  tickPosition = tickPositionRight = tickPositionLeft = coTickPosition = 0;
}





/******************************************************************************
 * isNew???()  Return true if new data has been read.
 **************************************************************************/
boolean isNewGyro() {
  if (digitalRead(GYRO_INTR_PIN) == LOW) return false;
  lsm6.readGyro();
  return true;
}
boolean isNewAccel() {
  if (digitalRead(ACCEL_INTR_PIN) == LOW) return false;
  lsm6.readAcc();
  return true;
}




/******************************************************************************
 * resetIMU()  From: https://forum.arduino.cc/index.php?topic=386269.0
 *             I2C clocks to make sure no slaves are hung in a read
 *             at startup
 **************************************************************************/
void resetIMU() {
  // Issue 20 I2C clocks to make sure no slaves are hung in a read
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(70, OUTPUT);
  pinMode(71, OUTPUT);
  digitalWrite(20, LOW);
  digitalWrite(70, LOW);
  for (int i = 0; i < 1000; i++)
  {
    digitalWrite(21, LOW);
    digitalWrite(71, LOW);
    delayMicroseconds(10);
    digitalWrite(21, HIGH);
    digitalWrite(71, HIGH);
    delayMicroseconds(10);
  }
}



/******************************************************************************
 * readFahr()  Read the Fahrenheit temperature from the gyro
 **************************************************************************/
float readFahr() {
  Wire.beginTransmission(0b1101011);
  // automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
  Wire.write(LSM6::OUT_TEMP_L);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t) 0b1101011, (uint8_t) 2);

  while (Wire.available() < 2) {
  }

  uint8_t tl = Wire.read();
  uint8_t th = Wire.read();

  // combine high and low bytes
  int16_t out = (int16_t)(th << 8 | tl);
  float ret = ((float) out) / 16;
  ret += 25.0;
  ret = ((ret * 9.0) / 5) + 32;
  return ret;
}



const int DRIFT_SIZE = 102;
int zArray[DRIFT_SIZE];
int yArray[DRIFT_SIZE];
int xArray[DRIFT_SIZE];
/******************************************************************************
 * doGyroDrift()  Called 204/sec.  Average gyroDrift for x, y & z
 *                for one second periods.
 **************************************************************************/
void doGyroDrift() {
  static int gPtr = 0;
  int x, y, z;

  xArray[gPtr] = lsm6.g.x;
  yArray[gPtr] = lsm6.g.y;
  zArray[gPtr] = lsm6.g.z;
  gPtr++;
  if (gPtr >= DRIFT_SIZE) {
    gPtr = 0;
    int xSum = 0;
    int ySum = 0;
    int zSum = 0;
    int xMax = xArray[0];
    int xMin = xMax;
    int yMax = yArray[0];
    int yMin = yMax;
    int zMax = zArray[0];
    int zMin = zMax;
    for (int i = 0; i < DRIFT_SIZE; i++) {
      x = xArray[i];
      if (x > xMax)  xMax = x;
      if (x < xMin)  xMin = x;
      xSum += x;
      y = yArray[i];
      if (y > yMax)  yMax = y;
      if (y < yMin)  yMin = y;
      ySum += y;
      z = zArray[i];
      if (z > zMax)  zMax = z;
      if (z < zMin)  zMin = z;
      zSum += z;
    }
    float xAve = ((float) xSum) / ((float) DRIFT_SIZE);
    float yAve = ((float) ySum) / ((float) DRIFT_SIZE);
    float zAve = ((float) zSum) / ((float) DRIFT_SIZE);

    // If we have a stable 0.5 second period, average the most recent 20 periods & adjust drift.
    if (((xMax - xMin) < 30) && ((yMax - yMin) < 30) && ((zMax - zMin) < 30)) {
      setDrift(xAve, yAve, zAve);
    }
//    sprintf(message, "xMin: %4d     xMax: %4d     xAve: %5.1f   ", xMax, xMin, xAve);
//    Serial.print(message);
//    sprintf(message, "yMin: %4d     yMax: %4d     yAve: %5.1f   ", yMax, yMin, yAve);
//    Serial.print(message);
//    sprintf(message, "zMin: %4d     zMax: %4d     zAve: %5.1f\n", zMax, zMin, zAve);
//    Serial.print(message);
  }
}

#define AVE_SIZE 20
void setDrift(float xAve, float yAve, float zAve) {
  static float xAveArray[AVE_SIZE];
  static float yAveArray[AVE_SIZE];
  static float zAveArray[AVE_SIZE];
  static int aveTotal = 0;
  static int avePtr = 0;

  float sumXAve = 0.0;
  float sumYAve = 0.0;
  float sumZAve = 0.0;

  xAveArray[avePtr] = xAve;
  yAveArray[avePtr] = yAve;
  zAveArray[avePtr] = zAve;

  avePtr++;
  avePtr = avePtr % AVE_SIZE;
  if (aveTotal < avePtr) aveTotal = avePtr;

  for (int i = 0; i < aveTotal; i++) {
    sumXAve += xAveArray[i];
    sumYAve += yAveArray[i];
    sumZAve += zAveArray[i];
  }
  float aveXDrift = sumXAve / aveTotal;
  float aveYDrift = sumYAve / aveTotal;
  float aveZDrift = sumZAve / aveTotal;
  timeDriftPitch = aveXDrift;
  timeDriftRoll = aveYDrift;
  timeDriftYaw = aveZDrift;
//    sprintf(message, "gPitch: %5.2f     gRoll: %5.2f     gYaw: %5.2f\n", gPitch, gRoll, gYaw);
//    Serial.print(message);
  
}
