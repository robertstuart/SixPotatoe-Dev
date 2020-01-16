/*****************************************************************************-
 *                        Motor.ino
 *****************************************************************************/


/******************************************************************************
 *  motorInit()
 *****************************************************************************/
void motorInit() {
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  pinMode(ENC_A_RIGHT_PIN, INPUT);
  pinMode(ENC_A_LEFT_PIN, INPUT);
  pinMode(ENC_B_RIGHT_PIN, INPUT);
  pinMode(ENC_B_LEFT_PIN, INPUT);

  analogWriteFrequency(PWM_RIGHT_PIN, 20000);
  analogWriteFrequency(PWM_LEFT_PIN, 20000);

  digitalWrite(DIR_RIGHT_PIN, LOW);
  digitalWrite(DIR_LEFT_PIN, LOW);
  analogWrite(PWM_RIGHT_PIN, 0);
  analogWrite(PWM_LEFT_PIN, 0);

  attachInterrupt(digitalPinToInterrupt(ENC_A_RIGHT_PIN), encoderIsrRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_LEFT_PIN), encoderIsrLeft, CHANGE);
}



/******************************************************************************
 * encoderIsr???()
 *****************************************************************************/
void encoderIsrRight() {
  static boolean encAStat;
  static boolean encBStat;
  long tickPeriodRight;

  boolean encA = (digitalReadFast(ENC_A_RIGHT_PIN) == HIGH) ? true : false;
  if (encA == encAStat) return;  // Ignore bogus interrupts
  encAStat = encA;
  unsigned long lastTickTime = tickTimeRight;
  tickTimeRight = micros();
  boolean encB = (digitalReadFast(ENC_B_RIGHT_PIN) == HIGH) ? true : false;
  if (encB == encBStat) return;  // Ignore reversal of direction
  encBStat = encB;

  // Set the speed & tickPosition
  if (encA == encB) {
    tickPeriodRight = (long) tickTimeRight - (long) lastTickTime;
    tickPositionRight++;
  }
  else {
    tickPeriodRight = (long) lastTickTime - (long) tickTimeRight;
    tickPositionRight--;
  }
  //  int mFpsRight = (ENC_FACTOR_M / tickPeriodRight); // speed in milli-fps
  //  fpsSumRight += mFpsRight;
  //  fpsCountRight++;
  tickSumRight += tickPeriodRight;
  tickCountRight++;
} // encoderIsrRight()


/**************************************************************************.
   encoderIsrLeft()
 **************************************************************************/
void encoderIsrLeft() {
  static boolean encAStat;
  static boolean encBStat;
  long tickPeriodLeft;

  boolean encA = (digitalReadFast(ENC_A_LEFT_PIN) == HIGH) ? true : false;
  if (encA == encAStat) return;  // Ignore bogus interrupts
  encAStat = encA;
  unsigned long lastTickTime = tickTimeLeft;
  tickTimeLeft = micros();
  boolean encB = (digitalReadFast(ENC_B_LEFT_PIN) == HIGH) ? true : false;
  if (encB == encBStat) return;  // Ignore reversal of direction.
  encBStat = encB;

  // Set the speed & tickPosition
  if (encA == encB) {
    tickPeriodLeft = (long) lastTickTime - (long) tickTimeLeft;
    tickPositionLeft--;
  }
  else {
    tickPeriodLeft = (long)tickTimeLeft - (long) lastTickTime;
    tickPositionLeft++;
  }
  //  int mFpsLeft = (ENC_FACTOR_M / tickPeriodLeft); // speed in milli-fps
  //  fpsSumLeft += mFpsLeft;
  //  fpsCountLeft++;
  tickSumLeft += tickPeriodLeft;
  tickCountLeft++;
} // end encoderIsrLeft();



/******************************************************************************
   readSpeed????()  Called every loop from CheckMotor???
 *****************************************************************************/
void readSpeedRight() {
  noInterrupts();
  int sum = tickSumRight;
  int count =  tickCountRight;
  tickSumRight = 0L;
  tickCountRight = 0;
  interrupts();
  if (count == 0) {
    int newMFps = ENC_FACTOR_M / (micros() - tickTimeRight);
    if (wMFpsRight < 0) newMFps = -newMFps;
    if (newMFps > 0) {
      if (newMFps < wMFpsRight) wMFpsRight = newMFps; // Set new if lower
    } else {
      if (newMFps > wMFpsRight) wMFpsRight = newMFps; // Set new if lower
    }
  } else {
    wMFpsRight = (ENC_FACTOR_M * count) / sum;
  }
  wFpsRight = (wMFpsRight) / 1000.0;
}

void readSpeedLeft() {
  noInterrupts();
  long sum = tickSumLeft;
  int count =  tickCountLeft;
  tickSumLeft = 0L;
  tickCountLeft = 0;
  interrupts();
  if (count == 0) {
    int newMFps = ENC_FACTOR_M / (micros() - tickTimeLeft);
    if (wMFpsLeft < 0) newMFps = -newMFps;
    if (newMFps > 0) {
      if (newMFps < wMFpsLeft) wMFpsLeft = newMFps; // Set new if lower
    } else {
      if (newMFps > wMFpsLeft) wMFpsLeft = newMFps; // Set new if lower
    }
  }
  else {
    wMFpsLeft = (ENC_FACTOR_M * count) / sum;
  }
  wFpsLeft = ((float) wMFpsLeft) / 1000.0;
}


/******************************************************************************
   checkMotors()
 *****************************************************************************/
void checkMotors() {
  checkMotorRight();
  checkMotorLeft();
  wFps = (wFpsLeft + wFpsRight) / 2.0;
  wMFps = (wMFpsRight + wMFpsLeft) / 2;
}

/******************************************************************************
   checkMotor????()  Called every loop
 *****************************************************************************/
void checkMotorRight() {
  float motorGain = MOTOR_GAIN;
  readSpeedRight();

  float wsError = (float) (targetWFpsRight - wFpsRight);
  if (abs(targetWFpsRight) < 0.5) {  // reduce gain below .5 fps
    motorGain = 1.0 + (abs(targetWFpsRight) * 8.0);
  }
  float wsTarget = targetWFpsRight + (wsError * motorGain);  // Target speed to correct error
  float pw = abs(wsTarget * FPS_TO_PW) + DEAD_ZONE;            // Pw for the target.
  if (pw <= DEAD_ZONE) pw = 0;
  setMotorRight(pw, wsTarget > 0.0);
}

void checkMotorLeft() {
  float motorGain = MOTOR_GAIN;
  readSpeedLeft();

  float wsError = (float) (targetWFpsLeft - wFpsLeft);
  if (abs(targetWFpsLeft) < 0.5) {  // reduce gain below .5 fps
    motorGain = 1.0 + (abs(targetWFpsLeft) * 8.0);
  }
  float wsTarget = targetWFpsLeft + (wsError * motorGain);  // Target speed to correct error
  float pw = abs(wsTarget * FPS_TO_PW) + DEAD_ZONE;            // Pw for the target.
  if (pw <= DEAD_ZONE) pw = 0;
  setMotorLeft(pw, wsTarget > 0.0);
}


/*******************************************************************************
   setMotor????() Set pw and diriction. pw between 0-255
 ******************************************************************************/
void setMotorRight(int pw, bool isFwd) {
  if (!isRunning) pw = 0;
  if (pw > 255) pw = 255;
  else if (pw < 0) pw = 0;
  digitalWrite(DIR_RIGHT_PIN, (isFwd) ? LOW : HIGH);
  analogWrite(PWM_RIGHT_PIN, pw);
}

void setMotorLeft(int pw, bool isFwd) {
  if (!isRunning) pw = 0;
  if (pw > 255) pw = 255;
  else if (pw < 0) pw = 0;
  digitalWrite(DIR_LEFT_PIN, (isFwd) ? HIGH : LOW);
  analogWrite(PWM_LEFT_PIN, pw);
}
