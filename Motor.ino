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
  if (encA != encB) {
    tickPeriodRight = (long) tickTimeRight - (long) lastTickTime;
    tickPositionRight++;
  }
  else {
    tickPeriodRight = (long) lastTickTime - (long) tickTimeRight;
    tickPositionRight--;
  }
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
  if (encA != encB) {
    tickPeriodLeft = (long) lastTickTime - (long) tickTimeLeft;
    tickPositionLeft--;
  }
  else {
    tickPeriodLeft = (long)tickTimeLeft - (long) lastTickTime;
    tickPositionLeft++;
  }
  tickSumLeft += tickPeriodLeft;
  tickCountLeft++;
} // end encoderIsrLeft();



/******************************************************************************
   readSpeed????()  Called every loop from CheckMotor()
 *****************************************************************************/
void readSpeedRight() {
  noInterrupts();
  int sum = tickSumRight;
  int count =  tickCountRight;
  tickSumRight = 0L;
  tickCountRight = 0;
  interrupts();
  if (count == 0) {
    float newWKph = USEC_TO_KPH / ((float) (micros() - tickTimeRight));
    if (wKphRight < 0.0) newWKph = -newWKph;
    if (newWKph > 0.0) {
      if (newWKph < wKphRight) wKphRight = newWKph; // Set new if lower
    } else {
      if (newWKph > wKphRight) wKphRight = newWKph; // Set new if lower
    }
  } else {
    wKphRight =  (USEC_TO_KPH * ((float) count)) / ((float) sum) ;
  }
}

void readSpeedLeft() {
  noInterrupts();
  long sum = tickSumLeft;
  int count =  tickCountLeft;
  tickSumLeft = 0L;
  tickCountLeft = 0;
  interrupts();
  if (count == 0) {
    float newWKph = USEC_TO_KPH / ((float) (micros() - tickTimeLeft));
    if (wKphLeft < 0.0) newWKph = -newWKph;
    if (newWKph > 0.0) {
      if (newWKph < wKphLeft) wKphLeft = newWKph; // Set new if lower
    } else {
      if (newWKph > wKphLeft) wKphLeft = newWKph; // Set new if lower
    }
  }
  else {
    wKphLeft = (USEC_TO_KPH * ((float) count)) / ((float) sum);
  }
}

 

/******************************************************************************
   runMotors()
 *****************************************************************************/
void runMotors() {
  runMotorRight();
  runMotorLeft();
  wKph = (wKphLeft + wKphRight) / 2.0;
  tickPosition = tickPositionRight + tickPositionLeft;
  tickMeters = tickPosition / (TICKS_PER_METER * 2.0);
}



/******************************************************************************
 * runMotor????()  Called every loop
 *****************************************************************************/
void runMotorRight() {
  float motorGain = MOTOR_GAIN;
  readSpeedRight();

  float wsError = (float) (targetWKphRight - wKphRight);
  if (abs(targetWKphRight) < 0.5) {  // reduce gain below .5 fps
    motorGain = 1.0 + (abs(targetWKphRight) * 8.0);
  }
  motorTargetKphRight = targetWKphRight + (wsError * motorGain);  // Target speed to correct error
  float pw = abs(motorTargetKphRight * KPH_TO_PW);            // Pw for the target.
  setMotorRight(pw, motorTargetKphRight > 0.0);
}

void runMotorLeft() {
  float motorGain = MOTOR_GAIN;
  readSpeedLeft();

  float wsError = (float) (targetWKphLeft - wKphLeft);
  if (abs(targetWKphLeft) < 0.5) {  // reduce gain below .5 fps
    motorGain = 1.0 + (abs(targetWKphLeft) * 8.0);
  }
  motorTargetKphLeft = targetWKphLeft + (wsError * motorGain);  // Target speed to correct error
  float pw = abs(motorTargetKphLeft * KPH_TO_PW);            // Pw for the target.
  setMotorLeft(pw, motorTargetKphLeft > 0.0);
}


/*******************************************************************************
   setMotor????() Set pw and diriction. pw between 0-255
 ******************************************************************************/
void setMotorRight(int pw, bool isFwd) {
  if (!isRunning) pw = 0;
  else if (pw > 255) pw = 255;
  else if (pw < 0) pw = 0;
  digitalWrite(DIR_RIGHT_PIN, (isFwd) ? LOW : HIGH);
  analogWrite(PWM_RIGHT_PIN, pw);\
  motorRightPw = pw;  // For panic
}

void setMotorLeft(int pw, bool isFwd) {
  if (!isRunning) pw = 0;
  else if (pw > 255) pw = 255;
  else if (pw < 0) pw = 0;
  digitalWrite(DIR_LEFT_PIN, (isFwd) ? HIGH : LOW);
  analogWrite(PWM_LEFT_PIN, pw);
  motorLeftPw = pw;
}
