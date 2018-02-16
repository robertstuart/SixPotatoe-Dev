
void motorInit() {
//  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
//  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//  //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  pinMode(ENC_A_RIGHT_PIN, INPUT);
  pinMode(ENC_A_LEFT_PIN, INPUT);
  pinMode(ENC_B_RIGHT_PIN, INPUT);
  pinMode(ENC_B_LEFT_PIN, INPUT);

  digitalWrite(DIR_RIGHT_PIN, LOW);
  digitalWrite(DIR_LEFT_PIN, LOW);
  analogWriteFrequency(PWM_RIGHT_PIN, 20000);
  analogWriteFrequency(PWM_LEFT_PIN, 20000);
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

  boolean encA = (digitalRead(ENC_A_RIGHT_PIN) == HIGH) ? true : false;
  if (encA == encAStat) return;  // Ignore bogus interrupts
  encAStat = encA;
  unsigned long lastTickTime = tickTimeRight;
  tickTimeRight = micros();
  boolean encB = (digitalRead(ENC_B_RIGHT_PIN) == HIGH) ? true : false;
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

  boolean encA = (digitalRead(ENC_A_LEFT_PIN) == HIGH) ? true : false;
  if (encA == encAStat) return;  // Ignore bogus interrupts
  encAStat = encA;
  unsigned long lastTickTime = tickTimeLeft;
  tickTimeLeft = micros();
  boolean encB = (digitalRead(ENC_B_LEFT_PIN) == HIGH) ? true : false;
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



/*******************************************************************************
 * setMotor????() Set pw and diriction. pw between 0-255
 ******************************************************************************/
void setMotorRight(int pw, bool isFwd) {
  if (!isRunning) pw = 0;
  if (pw > 255) pw = 255;
  else if (pw < 0) pw = 0;
  digitalWrite(DIR_RIGHT_PIN, (isFwd) ? HIGH : LOW);
  analogWrite(PWM_RIGHT_PIN, pw);
}

void setMotorLeft(int pw, bool isFwd) {
  if (!isRunning) pw = 0;
  if (pw > 255) pw = 255;
  else if (pw < 0) pw = 0;
  digitalWrite(DIR_LEFT_PIN, (isFwd) ? LOW : HIGH);
  analogWrite(PWM_LEFT_PIN, pw);
}
