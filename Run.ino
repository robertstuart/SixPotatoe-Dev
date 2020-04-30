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
      else rcControl();

      setCoKph();
      if (isBowlBalancing && isRouteInProgress) bowlBalance();
      else if (isAir && isRouteInProgress) ; // Do nothing. Set by doAir().
      else if (isUpright) balance();
      
      runMotors();
      blink13();
      updateCartesian();
      watchdog();
      postLog();
//if (isRunning) log(imu.maPitch, isUpright, controllerY, wKph);
    }
}



/*****************************************************************************-
 *  rcControl() Control speed & steering from RC controller
 *****************************************************************************/
void rcControl() {
  // Set values for balancing
  balanceTargetKph = controllerY * K30;
  float yfac = (((1.0 - abs(controllerY)) * 01.5) + 0.5) * 2.0; //wKph rather than controllerY?
  balanceSteerAdjustment = -yfac * controllerX; 
  
  // Set values for on the ground (!isUpright)
  float targetWKph = controllerY * K31;
  float steerDiff = controllerX * K32;
  targetWKphRight = targetWKph + steerDiff;
  targetWKphLeft = targetWKph - steerDiff;
}



/*****************************************************************************-
 *  balance() 
 *****************************************************************************/
void balance() {

  // Find the speed error.
  float coKphError = balanceTargetKph - coKph;

  // compute a weighted angle to eventually correct the speed error
  targetPitch = -(coKphError * K5); // Speed error to angle 
  
  // Compute maximum angles for the current wheel speed and enforce limits.
  targetPitch = constrain(targetPitch, -K12, K12);

  // Compute angle error and weight factor
  angleError = targetPitch - imu.maPitch;
  angleError = constrain(angleError, -K13, K13); // prevent "jumping"
  kphCorrection = angleError * K14; // Angle error to speed (Kph) 
  float d = imu.gyroPitchDelta *  K17; // add "D" to reduce overshoot

  // Reduce D to zero at K15
  float ratio = (K15 - abs(imu.maPitch)) / K15;
  ratio = constrain(ratio, 0.0, 1.0);
  kphCorrection -= (d * ratio);
//  kphCorrection -= d;

  // Reduce the correction after zeroG.

  // Add the angle error to the base speed to get the target wheel speed.
  targetWKph = kphCorrection + coKph;

  // Keep at same wheel speed if at Zero G
//  if (isZeroG) targetWKph = zeroGKph;

  targetWKphRight = targetWKph - balanceSteerAdjustment;
  targetWKphLeft = targetWKph + balanceSteerAdjustment;

//if (isRunning) addLog(imu.vertAccel, imu.accelZ, wKph, imu.maPitch);
//Serial.println(wKph);
//if (abs(wKph) > 3.0) Serial.print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
} // end balance() 



/*****************************************************************************-
 *  bowlBalance() 
 *****************************************************************************/
void bowlBalance() {

  // Compute angle error and weight factor
  angleError = bowlTargetPitch - imu.maPitch;
  kphCorrection = angleError * K14; // Angle error to speed 
  float d = imu.gyroPitchDelta *  0.1; // add "D" to reduce overshoot
  kphCorrection -= d;

  // Add the angle error to the base speed to get the target wheel speed.
  targetWKph = kphCorrection + coKph;

  // Adjust wheel speeds to keep roll at zero at end of bowl.
  float steerAdjustment = K8 * bowlCompleted * imu.maRoll;
//  addLog(steerAdjustment, targetWKph, bowlCompleted, imu.maRoll);
//  steerAdjustment = 0.0;

  // Keep at same wheel speed if at Zero G
  if (isZeroG) targetWKph = zeroGKph;

  targetWKphRight = targetWKph + steerAdjustment;
  targetWKphLeft = targetWKph - steerAdjustment;
//addLog(targetWKph, wKph, bowlTargetPitch, imu.maPitch);
//addLog(imu.maPitch, currentDistance, error, 0.0);
} // end bowlBalance() 



/*****************************************************************************-
 *  setCoKPh() Set the center of oscillation speed (cos) which is determined 
 *             by accelerometer and/or wheel speed.
 *****************************************************************************/
void setCoKph() {
  static float wheelCoKphOld = 0.0;
//  static const int COS_BUF_SIZE = 3;
//  static float cosBuff[COS_BUF_SIZE];
//  static int cosBuffPtr = 0;

//  isZeroG = (imu.vertAccel < 0.5) ? true : false;
//
//  if (isAir) {
//    zeroGKph = wheelCoKphOld = wKph;
//  } else if (isZeroG) {
//    coKph = zeroGKph;  // no-op, not used?
//  } else {  // Compute cos using wheel speed.
    float delta = imu.gyroPitchDelta * cos(DEG_TO_RAD * imu.maPitch);
    float rotation = delta * K1;  // ~1.6
    float wheelCoKph = wKph - rotation;
    wheelCoKph = (wheelCoKphOld * (1 - K2)) + (wheelCoKph * K2); //  
    wheelCoKphOld = wheelCoKph;
    coKph = wheelCoKph;
//    zeroGKph = cosBuff[cosBuffPtr]; // In case falling on next loop.
//    cosBuffPtr++;
//    cosBuffPtr = cosBuffPtr % COS_BUF_SIZE; 
//    cosBuff[cosBuffPtr] = wKph;
//  }
}



/*****************************************************************************-
 *  watchdog() Just toggle to keep watchdog timer alive
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
  const int MOD = 1;  // 1 = every loop, 2 = every other loop, mod3 = every 3rd loop, etc.
  logLoop++;
  if ((logLoop % MOD) == 0) { 
    if (isRunning) 
      addLog(imu.maPitch, imu.accelZ, imu.accelY, wKph);
  }
}
