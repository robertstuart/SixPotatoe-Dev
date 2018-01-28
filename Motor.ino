
void motorInit() {
  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  pinMode(ENC_A_RIGHT_PIN, INPUT);
  pinMode(ENC_A_LEFT_PIN, INPUT);
  pinMode(ENC_B_RIGHT_PIN, INPUT);
  pinMode(ENC_B_LEFT_PIN, INPUT);

  digitalWrite(DIR_RIGHT_PIN, LOW);
  digitalWrite(DIR_LEFT_PIN, LOW);
  analogWrite(PWM_RIGHT_PIN, 0);
  analogWrite(PWM_LEFT_PIN, 0);

  attachInterrupt(digitalPinToInterrupt(ENC_A_RIGHT_PIN), encoderIsrRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_LEFT_PIN), encoderIsrLeft, CHANGE);
}



/**************************************************************************.
   encoderIsr???()
 **************************************************************************/
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
 * encoderIsrLeft()
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



const float MOTOR_GAIN = 5.0;
const float FPS_TO_PW = 255.0 / 14.1; // change in PW gives change of 1.0 FPS, 612 rpm servocity motor
/**************************************************************************.
 * checkMotor????()  Called 200/sec
 **************************************************************************/
void checkMotorRight() {
  float motorGain = MOTOR_GAIN;
  readSpeedRight();

  float wsError = (float) (targetWhFpsRight - whFpsRight);       // Wheel speed error
  if (abs(targetWhFpsRight) < 1.5) {  // reduce gain below 1.5 fps
    motorGain = 0.5 + (abs(targetWhFpsRight) * 3.0);
  }
  float wsTarget = targetWhFpsRight + (wsError * motorGain);  // Target speed to correct error
  float pw = abs(wsTarget * FPS_TO_PW);            // Pw for the target.
  bool direction = (wsTarget > 0.0) ? true : false;
  setMotorRight(pw, direction);
}

void checkMotorLeft() {
  float motorGain = MOTOR_GAIN;
  readSpeedLeft();

  float wsError = (float) (targetWhFpsLeft - whFpsLeft);       // Wheel speed error
  if (abs(targetWhFpsLeft) < 1.5) {
    motorGain = 1.0 + (abs(targetWhFpsLeft) * 3.0);
  }
  float wsTarget = targetWhFpsLeft + (wsError * motorGain);  // Target speed to correct error
  float pw = abs(wsTarget * FPS_TO_PW);            // Pw for the target.
  boolean direction = (wsTarget > 0.0) ? true : false;
  setMotorLeft(pw, direction);
}



/**************************************************************************.
 * readSpeed????()  Called from CheckMotor???
 **************************************************************************/
void readSpeedRight() {
  noInterrupts();
  long sum = tickSumRight;
  long count =  tickCountRight;
  tickSumRight = 0L;
  tickCountRight = 0L;
  interrupts();
  if (count == 0) {
    long newMFps = ENC_FACTOR_M / (micros() - tickTimeRight);
    if (newMFps < abs(whMFpsRight)) {
      if (whMFpsRight > 0) {
        whMFpsRight = newMFps;
      } else {
        whMFpsRight = -newMFps;
      }
    }
  } else {
    whMFpsRight = ENC_FACTOR_M / (sum / count);
  }
  whFpsRight = ((float)whMFpsRight) / 1000.0;
}

void readSpeedLeft() {
  noInterrupts();
  long sum = tickSumLeft;
  long count =  tickCountLeft;
  tickSumLeft = 0L;
  tickCountLeft = 0L;
  interrupts();
  if (count == 0) {
    long newMFps = ENC_FACTOR_M / (micros() - tickTimeLeft);
    if (newMFps < abs(whMFpsLeft)) {
      if (whMFpsLeft > 0) {
        whMFpsLeft = newMFps;
      } else {
        whMFpsLeft = -newMFps;
      }
    }
  } else {
    whMFpsLeft = ENC_FACTOR_M / (sum / count);
  }
  whFpsLeft = ((float) whMFpsLeft) / 1000.0;
}



/*******************************************************************************
 * checkMotors() called from main loop. 200/sec
 ******************************************************************************/
void checkMotors() {
  static unsigned int pollCount;
  checkMotorRight();
  checkMotorLeft();
  whFps = (whFpsLeft + whFpsRight) / 2.0;
}



/*******************************************************************************
 * setMotor????() Set pw and diriction. pw between 0-255
 ******************************************************************************/
void setMotorRight(int pw, bool isFwd) {
  if (!isRunning) pw = 0;
  if (pw > 255) pw = 255;
  else if (pw < 0) pw = 0;
  digitalWriteFast(DIR_RIGHT_PIN, (isFwd) ? HIGH : LOW)
  analogWrite(PWM_RIGHT_PIN, pw);
}

void setMotorLeft(int pw, bool isFwd) {
  if (!isRunning) pw = 0;
  if (pw > 255) pw = 255;
  else if (pw < 0) pw = 0;
  digitalWriteFast(DIR_LEFT_PIN, (isFwd) ? LOW : HIGH)
  analogWrite(PWM_LEFT_PIN, pw);
}
