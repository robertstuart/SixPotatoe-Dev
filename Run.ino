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
  delay(200); // For switches?
  setBlink(LED_PIN, 150, 150);
  while(true) { // main loop
    commonTasks();
    // Add code to timeout in imu read!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if (isNewImuData()) {
      if (isGettingUp) gettingUp();
      else balance(); 
      checkMotors();
      safeAngle();
    } 
  }
}



const double ZERO_ANGLE = 0.5;
/***********************************************************************.
 *  balance() 
 ***********************************************************************/
void balance() {
       
  // Compute Center of Oscillation speed (cos)
  rotation3 = -gyroPitchDelta * CONST_COS_ROTATION;  // 4.5
rotation3 = 0.0;
  cos3 = wFps + rotation3;
  // 0.92 .u value: 0.0 = no hf filtering, large values give slow response
  coFps = (lpfCos3Old * CONST_COS_LPF) + (cos3 * (1.0D - CONST_COS_LPF));
  lpfCos3Old = coFps;

  // Get the controller target speed.
  targetCoFps = controllerY * MAX_FPS * 0.01;

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

  speedAdjustment = (((100.0 - abs(controllerY)) * 0.015) + 0.5) * ((float) controllerX * 0.01); 
  targetWFpsRight = targetWFps - speedAdjustment;
  targetWFpsLeft = targetWFps + speedAdjustment;

} // end balance() 



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
