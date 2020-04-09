/*****************************************************************************-
 *                           Run.ino 
 *****************************************************************************/
float kphCorrection = 0.0f;
float angleError = 0.0;
float targetPitch = 0.0;
float kphAdjustment = 0.0;


/*****************************************************************************-
 *  run() Continuous loop for doing all tasks.
 *****************************************************************************/
void run() {
    commonTasks();
    if (imu.isNewImuData()) {
      if (isRouteInProgress) routeControl(); 
      else manualControl();
      if (isBalancing) balance();
      runMotors();
      blink13();
      watchdog();
      postLog();
      updateCartesian();
    }
}



/*****************************************************************************-
 *  manualControl() Control speed & steering from RC
 *****************************************************************************/
void manualControl() {
  // Set values for balancing
  balanceTargetKph = controllerY * MAX_BALANCE_KPH;
  float yfac = (((1.0 - abs(controllerY)) * 01.5) + 0.5) * 2.0; //wKph rather than controllerY?
  balanceSteerAdjustment = -yfac * controllerX; 
  
  // Set values for on the ground
  float targetWKph = controllerY * MAX_GROUND_KPH;
  float steerDiff = controllerX * MAX_GROUND_STEER;
  targetWKphRight = targetWKph + steerDiff;
  targetWKphLeft = targetWKph - steerDiff;
}



/*****************************************************************************-
 *  balance() 
 *****************************************************************************/
void balance() {
  coKph = getCoKph();

  // Find the speed error.  Constrain rate of change?
  float coKphError = balanceTargetKph - coKph;

  // compute a weighted angle to eventually correct the speed error
  targetPitch = -(coKphError * CONST_ERROR_TO_ANGLE); //** 4.0 ******** Speed error to angle *******************
  
  // Compute maximum angles for the current wheel speed and enforce limits.
  targetPitch = constrain(targetPitch, -K12, K12);

  // Compute angle error and weight factor
  angleError = targetPitch - imu.maPitch;
  angleError = constrain(angleError, -K13, K13); // prevent "jumping"
  kphCorrection = angleError * K14; // Angle error to speed 
  float d = imu.gyroPitchDelta *  0.1; // add "D" to reduce overshoot
  kphCorrection -= d;

  // Add the angle error to the base speed to get the target wheel speed.
  targetWKph = kphCorrection + coKph;

  targetWKphRight = targetWKph - balanceSteerAdjustment;
  targetWKphLeft = targetWKph + balanceSteerAdjustment;

} // end balance() 



/*****************************************************************************-
 *  getCoKPh() Return center of oscillation speed (cos) which is determined 
 *             by accelerometer and/or wheel speed.
 *****************************************************************************/
float getCoKph() {
  static float wheelCoKphOld = 0.0;
  static float accelCoKph = 0.0;
  static const int COS_BUF_SIZE = 20;
  static float cosBuff[COS_BUF_SIZE];
  static int cosBuffPtr = 0;
  static float fallingCoKph;

  float vertAccel = (cos(imu.maPitch * DEG_TO_RAD) * imu.accelZ) + (sin(imu.maPitch * DEG_TO_RAD) * imu.accelY);
  float horAccel = (sin(imu.maPitch * DEG_TO_RAD) * imu.accelZ) + (cos(imu.maPitch * DEG_TO_RAD) * imu.accelY);

  if (vertAccel > 0.5) {
    
    // Compute cos using wheel speed.
    float delta = imu.gyroPitchDelta * cos(DEG_TO_RAD * imu.maPitch);
    float rotation = delta * K1;  // ~1.6
    float wheelCoKph = wKph - rotation;
    wheelCoKph = (wheelCoKphOld * (1 - K2)) + (wheelCoKph * K2); // 0.92?, 
    wheelCoKphOld = wheelCoKph;

    // Compute cos using accelerometer
    accelCoKph -= (horAccel * ACCEL_TO_KPH);
    if (vertAccel > 0.8) {
      accelCoKph = (accelCoKph * K21) + (wheelCoKph * (1 - K21));
    }

    float cos = wheelCoKph; // Change to accelCoKph if using acceleromenter.
    if (vertAccel > 0.8) {
      fallingCoKph = cosBuff[cosBuffPtr]; // In case falling on next loop.
      cosBuffPtr = ++cosBuffPtr % COS_BUF_SIZE; 
      cosBuff[cosBuffPtr] = cos;
    }
    
//  if (isRunning) log(imu.maPitch, vertAccel, cos, 1.0);
    return cos; 
  } else {
  if (isRunning) log(imu.maPitch, vertAccel, fallingCoKph, 0.0);
    return fallingCoKph;
  }
}



/*****************************************************************************-
 *  watchdog() Just toggle to keep watchdog alive
 *****************************************************************************/
void watchdog() {
  static bool toggle = false;
  toggle = !toggle;
  digitalWrite(WATCHDOG_PIN, toggle ? LOW : HIGH);
}



/*****************************************************************************-
 *  postLog() Called 200 times/sec.
 *****************************************************************************/
void postLog() {
  static unsigned int logLoop = 0;
  logLoop++;
  if ((logLoop % 1) == 0) { // 1 = every loop, 2 = every other loop, mod3 = every 3rd loop, etc.
//    if (isRunReady)
//      log(imu.accelY, imu.horizAccel, imu.horizSpeed/-4.7, wKph);
  }
}
