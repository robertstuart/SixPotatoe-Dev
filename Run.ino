/*****************************************************************************-
 *                           Run.ino 
 *****************************************************************************/
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


/*****************************************************************************-
 *  run() Continuous loop for doing all tasks.
 *****************************************************************************/
void run() {
  while(true) { // main loop
    commonTasks();
    if (imu.isNewImuData()) { 
      updateCartesian();
      if (isGettingUp) gettingUp(false);
      else balance(); 
      runMotors();
      blink13();
      watchdog();
    } 
  }
}



/*****************************************************************************-
 *  balance() 
 ***********************************************************************/
void balance() {
  // Compute Center of Oscillation speed (cos)
  float delta = imu.gyroPitchDelta * cos(DEG_TO_RAD * imu.maPitch); // account for pitch
  rotation = delta * K1;  // ~1.6
  coKph = wKph - rotation;
  // 0.92?, 1.0 = no hf filtering, small values give slow response
  coKph = (coKphOld * (1 - K2)) + (coKph * K2);
  coKphOld = coKph;

  // Get the controller target speed.
  targetCoKph = (isRouteInProgress) ? routeKph : controllerY * MAX_KPH;

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
//  kphCorrection += imu.gyroPitchDelta *  0.2; // add "D" to reduce overshoot

  // Add the angle error to the base speed to get the target wheel speed.
  targetWKph = kphCorrection + coKph;

  if (isRouteInProgress) steerRoute(); else steerRC();

  static bool isLogging = false;
  static unsigned long endLogT = 0UL;
  if (isGotUp || isLogging) {
    logHeader = "maPitch, kphCorrection, gyroPitchDelta, coKph";
    log(imu.maPitch, kphCorrection, imu.gyroPitchDelta, coKph);
    if (isGotUp) {
      isGotUp = false;
      endLogT = timeMilliseconds + 1500;
      isLogging = true;
    } else {
      if (timeMilliseconds > endLogT) {
        isLogging = false;
      }
    }
  }
} // end balance() 



/*****************************************************************************-
 *  steerRC()
 *****************************************************************************/
//void steerRC() {
////float k = 0.2;
////targetWKphRight = targetWKph * (1.0 + k);
////targetWKphLeft = targetWKph * (1.0 - k);;
//
//  float diff = (0.75 * controllerX)/ (1.0 - controllerX);
//  float radius = 0.15 / diff;
//  float maxRadius = (coKph * coKph)/125;
//  float maxDiff = 0.15 / maxRadius;
////  if (abs(diff) > maxDiff) {
////    if (coKph > 0.0) diff = maxDiff;
////    else diff = -maxDiff;
////  }
//  targetWKphRight = targetWKph * (1.0 + diff);
//  targetWKphLeft = targetWKph * (1.0 - diff);
//}

void steerRC() {
  float yfac = (((1.0 - abs(controllerY)) * 01.5) + 0.5) * 2.0;
  kphAdjustment = -yfac * controllerX; 
  targetWKphRight = targetWKph - kphAdjustment;
  targetWKphLeft = targetWKph + kphAdjustment;
}



/*****************************************************************************-
 *  watchdog() Just toggle to keep watchdog alive
 *****************************************************************************/
void watchdog() {
  static bool toggle = false;
  toggle = !toggle;
//  if (ch6Val > 0.0) {
    digitalWrite(WATCHDOG_PIN, toggle ? LOW : HIGH);
//  }
}



/*****************************************************************************-
 *  gettingUp()
 *****************************************************************************/
void gettingUp(bool reset) {
  static unsigned long gettingUpStartTime = 0UL;
  if (reset) {
    if(!isUpright) {
      isGotUp = true;
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
    }
  }
}



/*****************************************************************************-
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
