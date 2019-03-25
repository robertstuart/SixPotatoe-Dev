/******************************************************************************
 * Run.ino  Main loop for controlling balance and steering
 *****************************************************************************/
#define LOOP_TIMEOUT 10  // milliseconds max loop in case imu failure
double accelFps = 0.0;
double coAccelFps = 0.0;
double lpfAccelFps = 0.0;
double lpfTpFps = 0.0;
float fpsCorrection = 0.0f;
float fpsLpfCorrectionOld = 0.0;
float fpsLpfCorrection = 0.0;
float angleError = 0.0;
float targetAngle = 0.0;
float speedAdjustment = 0.0;


/******************************************************************************
 *  run() Continuous loop for doing all tasks.
 *****************************************************************************/
void run() {
  boolean isGyroRead = false;
  boolean isAccRead = false;
  timeMicroseconds = micros();
  timeMilliseconds = millis();
  delay(200); // For switches?
  setHeading(0.0D);
  resetTicks();
  beep(BEEP_UP);
  unsigned long timeoutTrigger = millis();
  while(true) { // main loop
    commonTasks();
    // Add code to timeout in imu read!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if (isNewGyro()) {
      setGyroData();
      isGyroRead = true;
    }
    readUp(); // Get latest.
    if (isNewAccel()) {
      setAccelData();
      isAccRead = true;
    }
    readUp();
    bool isTimeout = timeMilliseconds > timeoutTrigger;
    if (isTimeout) Serial.println("Imu timeout!");
    bool isImuRead = isGyroRead && isAccRead;
    if (isImuRead || isTimeout) {
      isGyroRead = isAccRead = false;
      timeoutTrigger = millis() + LOOP_TIMEOUT;
      if (isGettingUp) gettingUp();
      else if (isGettingDown) gettingDown();
      else tpAlgorithm(); 
      checkMotors();
      setNavigation();
      doGyroDrift();
      sendLog();
      safeAngle();
    } 
  }
}



//const int Y_BIAS = 0;
//const double Y_ONE_G = -16500.0;
//const double ACCEL_TO_SPEED = 0.000009;
//const double COMP_TC = 0.98;
//const double ACCEL_TC = 0.90;
//const float COMP_TC = 0.0; // Cos only
//const double FPS_ACCEL = 0.05;
const double ZERO_ANGLE = 0.5;
const double ONE_G_Y = -16530.0D;
const double ONE_G_Z = 17029.0D;
const double ACCEL_TO_FPS = 0.06;
/***********************************************************************.
 *  tpAlgorithm() 
 ***********************************************************************/
void tpAlgorithm() {
  if (!isHcActive) joyX = joyY = 0.0;
      
  static double lpfAccelFpsOld = 0.0;
//  static double tp7OldSpeedError = 0.0;
  static double oldAccelFps = 0.0D;
 
  // Compute Center of Oscillation speed (cos)
  rotation3 = -gyroPitchDelta * CONST_COS_ROTATION;  // 4.5
rotation3 = 0.0;
  cos3 = wFps + rotation3;
  // 0.92 .u value: 0.0 = no hf filtering, large values give slow response
  lpfCos3 = (lpfCos3Old * CONST_COS_LPF) + (cos3 * (1.0D - CONST_COS_LPF));
  lpfCos3Accel = lpfCos3 - lpfCos3Old;
  lpfCos3Old = lpfCos3;

  // Compute the acceleration speed
  double rad = (gaPitch + ZERO_ANGLE) * DEG_TO_RAD;
  double yG = ((double) lsm6.a.y) / ONE_G_Y;
  double zG = ((double) lsm6.a.z) / ONE_G_Z;
  double yAccel = -(((cos(rad) * yG) + (sin(rad) * zG)) * ACCEL_TO_FPS);
  accelFps += yAccel;

  if (!isRunning) accelFps = lpfAccelFps = 0.0;

  // Complementary filter with COS. Try to minimizen jumping & loss of traction effects.
  // 0.98, High value places more emphasis on accel.
  double accelFpsDelta = accelFps - oldAccelFps;
  oldAccelFps = accelFps;
  coAccelFps += accelFpsDelta;
  coAccelFps = (coAccelFps * CONST_ACCEL_LPF) + (lpfCos3 * (1.0 - CONST_ACCEL_LPF));

  // Choose which of the computations to use.  Uncomment just one.
  coFps = lpfCos3;

  // Get the controller target speed.
  targetCoFps = (isRouteInProgress) ?  routeFps : (joyY * MAX_FPS * 0.01);

  // Find the speed error.  Constrain rate of change.
  float coFpsError = targetCoFps - coFps;

  // compute a weighted angle to eventually correct the speed error
  targetAngle = -(coFpsError * CONST_ERROR_TO_ANGLE); //** 4.0 ******** Speed error to angle *******************
  
  // Compute maximum angles for the current wheel speed and enforce limits.
  targetAngle = constrain(targetAngle, -50.0, 50.0);

  // Compute angle error and weight factor
  angleError = targetAngle - gaPitch;
  fpsCorrection = angleError * CONST_ANGLE_TO_FPS; // 0.4 ******************* Angle error to speed *******************

  // Add the angle error to the base speed to get the target wheel speed.
  targetWFps = fpsCorrection + coFps;

  // These routines set the steering values.
  if (isRouteInProgress) steerRoute(targetWFps);
  else steer(targetWFps);

} // end aTp7() 



/***********************************************************************.
 *  steer() 
 ***********************************************************************/
void steer(float fp) {
  speedAdjustment = (((100.0 - abs(joyY)) * 0.015) + 0.5) * ((float) joyX * 0.01); 
  targetWFpsRight = fp - speedAdjustment;
  targetWFpsLeft = fp + speedAdjustment;
//  targetWFpsRight = fp;
//  targetWFpsLeft = fp;
}



/***********************************************************************.
 *  steerRoute() X value from Up is the speed difference between the wheels.
 ***********************************************************************/
void steerRoute(float fp) {
  targetWFpsRight = fp - routeFpsDiff;
  targetWFpsLeft = fp + routeFpsDiff;
}



/******************************************************************************
 *  setGetUp()
 *****************************************************************************/
void setGetUp() {
  if(!isUpright) {
    isRunReady = true;
    isGettingUp = true;
    gettingUpStartTime = timeMilliseconds;
  }
}


/******************************************************************************
 *  gettingUp()
 *****************************************************************************/
void gettingUp() {
  float tFps = 0.0;
  if ((gettingUpStartTime + 1000) < timeMilliseconds) {
    isGettingUp = false;
    isRunReady = false;
    return;
  }
  float ab = abs(gaPitch);
  if (ab < 7.0) {
    isGettingUp = false;
    return;
  }
  bool isBack = (gaPitch > 0.0) ? true : false;
  if ((gettingUpStartTime + 100) > timeMilliseconds) {
    tFps = -3.0; // Go back at start.
  } else {   
    if      (ab > 60.0) tFps = 4.0;
    else if (ab > 30.0) tFps = 4.0;
    else if (ab > 20.0) tFps = 4.0;
    else if (ab > 10.0) tFps = 3.0;
    else tFps = ab * 0.1;
  }
  if (isBack) tFps = -tFps;
  targetWFpsRight = targetWFpsLeft = tFps;
}



/******************************************************************************
 *  setGetDown()
 *****************************************************************************/
void setGetDown() {
  if (isUpright && isRunning) {
    gettingDownStartTime = timeMilliseconds;
    isGettingDown = true;
  }
}


/******************************************************************************
 *  gettingDown()
 *****************************************************************************/
void gettingDown() {
  
}


/***********************************************************************.
 *  sendLog() Called 208 times/sec.
 ***********************************************************************/
void sendLog() {
  static unsigned int logLoop = 0;
  logLoop++;
    
//  if (!(logLoop % 104)) log2PerSec();
//  if (!(logLoop % 21)) log10PerSec();
//  routeLog(); //  208/sec
//  if (!(logLoop % 10)) log20PerSec(); // 20/sec  
//  if (!(logLoop % 2)) log104PerSec(); // 104/sec  
//  if (isRouteInProgress  && isRunning)  log208PerSec();
//  log208PerSec();
}

void log2PerSec() {
//  Serial.print(isRouteInProgress);
//  Serial.print(isUpright); Serial.print(isRunReady); Serial.print(isMotorDisable); Serial.print("\t");
//  Serial.print(battVolt); Serial.print("\t");Serial.print(gPitch); Serial.print("\t");Serial.println(gPitch);
//  Serial.print(isRunning); Serial.print(isRouteInProgress); Serial.println(isRunReady);
}

void log10PerSec() {
  snprintf(message, MSG_SIZE, "gaPitch %4.2f   gHeading: %4.2f   gcHeading: %4.2f", gaPitch, gHeading, gcHeading);
  Serial.println(message);
}

void log20PerSec() {
//  sprintf(message,  "%5.2f\t%5.2f\t%5.2f\t", sonarLeft, sonarFront, sonarRight);
//  sendBMsg(SEND_MESSAGE, message); 
}

void log104PerSec() {
}


void log208PerSec() {
}

