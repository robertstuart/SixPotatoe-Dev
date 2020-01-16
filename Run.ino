/******************************************************************************
 *                           Run.ino 
 *****************************************************************************/
#define LOOP_TIMEOUT 10  // milliseconds max loop in case imu failure
double accelFps = 0.0;
double coAccelFps = 0.0;
double lpfAccelFps = 0.0;
double lpfTpFps = 0.0;
float kphCorrection = 0.0f;
float kphLpfCorrectionOld = 0.0;
float kphLpfCorrection = 0.0;
float angleError = 0.0;
float targetAngle = 0.0;
float speedAdjustment = 0.0;


/******************************************************************************
 *  run() Continuous loop for doing all tasks.
 *****************************************************************************/
void run() {


//  OpenLog myLog; //Create instance
//  myLog.begin(); //Open connection to OpenLog (no pun intended)
//  for (float i = 0.0; i < 9.1; i += 1.0) {
//    sprintf(message, "%.2f,%.2f,%.2f,%.2f,%.2f", i, sqrt(i), pow(i,1.01), i * .34, i);
//    Serial.println(message);
//    myLog.println(message);
//  }
//  myLog.syncFile();
//


  while(true) { // main loop
    commonTasks();
    // Add code to timeout in imu read!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if (isNewImuData()) { 
      if (isGettingUp) gettingUp();
      else balance(); 
      checkMotors();
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
  cos3 = wKph + rotation3;
  // 0.92 .u value: 0.0 = no hf filtering, large values give slow response
  coKph = (lpfCos3Old * CONST_COS_LPF) + (cos3 * (1.0D - CONST_COS_LPF));
  lpfCos3Old = coKph;

  // Get the controller target speed.
  targetCoKph = controllerY * MAX_KPH;

  // Find the speed error.  Constrain rate of change.
  float coKphError = targetCoKph - coKph;

  // compute a weighted angle to eventually correct the speed error
  targetAngle = -(coKphError * CONST_ERROR_TO_ANGLE); //** 4.0 ******** Speed error to angle *******************
  
  // Compute maximum angles for the current wheel speed and enforce limits.
  targetAngle = constrain(targetAngle, -50.0, 50.0);

  // Compute angle error and weight factor
  angleError = targetAngle - gaPitch;
  kphCorrection = angleError * CONST_ANGLE_TO_KPH; // 0.4 ******************* Angle error to speed *******************

  // Add the angle error to the base speed to get the target wheel speed.
  targetWKph = kphCorrection + coKph;

  speedAdjustment = (((100.0 - abs(controllerY)) * 0.015) + 0.5) * ((float) controllerX * 0.01); 
  targetWKphRight = targetWKph - speedAdjustment;
  targetWKphLeft = targetWKph + speedAdjustment;

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
  targetWKphRight = targetWKphLeft = tFps;
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
