/******************************************************************************
 *                           Run.ino 
 *****************************************************************************/
#define LOOP_TIMEOUT 10  // milliseconds max loop in case imu failure
//double accelFps = 0.0;
//double coAccelFps = 0.0;
//double lpfAccelFps = 0.0;
//double lpfTpFps = 0.0;
float kphCorrection = 0.0f;
//float kphLpfCorrectionOld = 0.0;
//float kphLpfCorrection = 0.0;
float angleError = 0.0;
float targetPitch = 0.0;
float kphAdjustment = 0.0;


/******************************************************************************
 *  run() Continuous loop for doing all tasks.
 *****************************************************************************/
void run() {
  while(true) { // main loop
    commonTasks();
    if (imu.isNewImuData()) { 
      if (isGettingUp) gettingUp(false);
//      else if (isStandingStill();
      else balance(); 
      runMotors();
      blink13();
    } 
  }
}

boolean isStandingStill() { return false; }

//const double ZERO_ANGLE = 0.5;
/***********************************************************************.
 *  balance() 
 ***********************************************************************/
void balance() {
  // Compute Center of Oscillation speed (cos)
  rotation = imu.gyroPitchDelta * CONST_COKPH_ROTATION;  // 4.5
  coKph = wKph - rotation;
//  float tc = (wKph < 0.6) ? 
  // 0.92?, 1.0 = no hf filtering, small values give slow response
  coKph = (coKphOld * (1 - CONST_COKPH_TC)) + (coKph * CONST_COKPH_TC);
  coKphOld = coKph;

  // Get the controller target speed.
  targetCoKph = controllerY * MAX_KPH;

  // Find the speed error.  Constrain rate of change?
  float coKphError = targetCoKph - coKph;

  // compute a weighted angle to eventually correct the speed error
  targetPitch = -(coKphError * CONST_ERROR_TO_ANGLE); //** 4.0 ******** Speed error to angle *******************
  
  // Compute maximum angles for the current wheel speed and enforce limits.
  targetPitch = constrain(targetPitch, -K12, K12);

  // Compute angle error and weight factor
  angleError = targetPitch - imu.maPitch;
  angleError = constrain(angleError, -K13, K13); // prevent "jumping"
  kphCorrection = angleError * CONST_ANGLE_TO_KPH; // 0.4 ******************* Angle error to speed *******************

  // Add the angle error to the base speed to get the target wheel speed.
  targetWKph = kphCorrection + coKph;
  
  float yfac = (((1.0 - abs(controllerY)) * 01.5) + 0.5) * 2.0;
  kphAdjustment = -yfac * controllerX; 
//  Serial.print(yfac); Serial.print("\t"); Serial.print(controllerX); Serial.print("\t"); Serial.println(speedAdjustment);
  targetWKphRight = targetWKph - kphAdjustment;
  targetWKphLeft = targetWKph + kphAdjustment;

  if (isRunning) {
    logHeader = "maPitch, wKph, targetWKphRight, gyroPitchDelta";
    log(imu.maPitch, wKph, targetWKphRight, imu.gyroPitchDelta);
  }
} // end balance() 



///******************************************************************************
// *  setGetUp()
// *****************************************************************************/
//void setGetUp() {
//  if(!isUpright) {
//    isRunReady = true;
//    isGettingUp = true;
//    gettingUpStartTime = timeMilliseconds;
//  }
//}
void gettingUp(bool reset) {
  static unsigned long gettingUpStartTime = 0UL;
  if (reset) {
    if(!isUpright) {
      isRunReady = true;
      isGettingUp = true;
      gettingUpStartTime = timeMilliseconds;
    }
  } else {
    if ((gettingUpStartTime + 50) > timeMilliseconds) {
      float tKph = -3.0; // Go backwards at start.
      if (imu.maPitch > 0.0) tKph = -tKph;
      targetWKphRight = targetWKphLeft = tKph;
    } else {
      isGettingUp = false;
//      isGetUp = true;  // Reset timer
    }
  }
}


///******************************************************************************
// *  gettingUp()
// *****************************************************************************/
//void gettingUp() {
//  float tKph = 0.0;
//  if ((gettingUpStartTime + 1000) < timeMilliseconds) {
//    isGettingUp = false;
//    isRunReady = false;
//    return;
//  }
//  float ab = abs(imu.maPitch);
//  if (ab < 10.0) {
//    isGettingUp = false;
//    return;
//  }
//  bool isBack = (imu.maPitch > 0.0) ? true : false;
//  if ((gettingUpStartTime + 100) > timeMilliseconds) {
//    tKph = -3.0; // Go backwards at start.
//  } else {   
//    if      (ab > 60.0) tKph = 6.0;
//    else if (ab > 30.0) tKph = 6.0;
//    else if (ab > 20.0) tKph = 6.0;
//    else if (ab > 10.0) tKph = 4.0;
//    else tKph = ab * 0.1;
//  }
//  if (isBack) tKph = -tKph;
//  targetWKphRight = targetWKphLeft = tKph;
//}
//


/***********************************************************************.
 *  sendLog() Called 200 times/sec.
 *      Set modcount to 1 to log every loop. Set to 100 to log 
 *      every 1/2 sec, etc.
 ***********************************************************************/
void sendLog() {
  static unsigned int modCount = 1;
  static unsigned int logLoop = 0;
  logLoop++;

  if ((modCount % 1) == 0) {
    // logging code here.
  }
  
}
