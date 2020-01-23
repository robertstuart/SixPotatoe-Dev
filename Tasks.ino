/*****************************************************************************-
 *                                 Task.ino
 *****************************************************************************/

 
/*****************************************************************************-
 * commonTasks()
 *****************************************************************************/
void commonTasks() {
  timeMicroseconds = micros();
  timeMilliseconds = millis();
  blinkLed();
  switches();
  safeAngle();
  setRunningState();
  blinkLed();
  checkLog();
}



/*****************************************************************************-
   setRunningState()
 *****************************************************************************/
void setRunningState() {
  static int oldCh4State = 0;
  static boolean isStartup = true;

  // Change run state SixPotatoe if there is a change in state on ch4
  if (((oldCh4State == 0) && (ch4State > 0)) && !isStartup) {
    isRunReady = true;
  }
  if ((oldCh4State > 0) && (ch4State == 0)) {
    isRunReady = false;
    isStartup = false;
  }
  if ((ch4State == 2) && (oldCh4State < 2)) {
    setGetUp();
  }
  oldCh4State = ch4State;

  // Set isRunning variable to control motors
  if (isRunReady && (isUpright || isGettingUp)) {
    isRunning = true;
    currentBlink = On;
  } else {
    isRunning = false;
    if (isRunReady) currentBlink = FastFlash;
    else currentBlink = SlowFlash;
  }
//  Serial.print(isRunning); Serial.print(" "); Serial.println(isRunning);
}



/*****************************************************************************-
   blinkLed()
 *****************************************************************************/
void blinkLed() {
  static unsigned long trigger = 0UL;
  static int blinkCount = 0;

  if (timeMilliseconds > trigger) {
    trigger = timeMilliseconds + 100;
    bool ledState = false;

    blinkCount++;
    switch (currentBlink) {
      case SlowFlash:
        ledState = ((blinkCount % 10) == 0);
        break;
      case FastFlash:
        ledState = ((blinkCount % 2) == 0);
        break;
      case SlowBlink:
        ledState = (((blinkCount / 5) % 2) == 0);
        break;
      case On:
        ledState = true;
        break;
      case Off:
      default:
        break;
    }
    digitalWrite(LED_A_PIN, ledState ? HIGH : LOW);
    digitalWrite(LED_PIN, ((blinkCount / 2) % 2) ? HIGH : LOW);  // Just blink the Teensy
  }
}



/*****************************************************************************-
   safeAngle() Check to see if we have fallen sidways or forwards.
      TODO make this dependent on speed
 *****************************************************************************/
void safeAngle() {
  static unsigned long tTime = 0UL; // time of last state change
  static boolean tState = false;  // Timed state. true = upright

  boolean cState = (abs(gaPitch) < 70.0); // Current real state
  if (!cState && tState) {
    tTime = timeMilliseconds; // Start the timer for a state change to fallen.
  } else if (!cState) {
    if ((timeMilliseconds - tTime) > 50) {
      isUpright = false;
    }
  } else {
    isUpright = true;
  }
  tState = cState;
}



/*****************************************************************************-
 * checkLog()
 ******************************************************************************/
void checkLog() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'd') {
      Serial.println("data");
    }
  }
}



/*****************************************************************************-
 * log()
 ******************************************************************************/
void log(String s) {
  dBuff[dBuffPtr] = s;
  dBuffPtr++;
  if (dBuffPtr >= DBUFF_SIZE) {
    dBuffPtr = 0;
    isDBuffFull = true;
  }
}



/*****************************************************************************-
 * switches()
 *      Check switches and debounce
 ******************************************************************************/
void switches() {
  static unsigned int timerA = 0;
  static boolean aState = false;
  static boolean oldAState = false;

  // Debounce switch A
  boolean a = digitalRead(SW_A_PIN) == LOW;
  if (a) timerA = timeMilliseconds;
  if ((timeMilliseconds - timerA) > 50) aState = false;
  else aState = true;

  // Switch A press transition
  if (aState && (!oldAState)) {
    isRunReady = !isRunReady;
  }
  oldAState = aState;
}





/*****************************************************************************-
    rcInit()
 *****************************************************************************/
void rcInit() {
  pinMode(CH1_RADIO_PIN, INPUT);
  pinMode(CH2_RADIO_PIN, INPUT);
  pinMode(CH3_RADIO_PIN, INPUT);
  pinMode(CH4_RADIO_PIN, INPUT);
  pinMode(CH5_RADIO_PIN, INPUT);
  pinMode(CH6_RADIO_PIN, INPUT);
  attachInterrupt(CH1_RADIO_PIN, ch1Isr, CHANGE);
  attachInterrupt(CH2_RADIO_PIN, ch2Isr, CHANGE);
  attachInterrupt(CH3_RADIO_PIN, ch3Isr, CHANGE);
  attachInterrupt(CH4_RADIO_PIN, ch4Isr, CHANGE);
  attachInterrupt(CH5_RADIO_PIN, ch5Isr, CHANGE);
  attachInterrupt(CH6_RADIO_PIN, ch6Isr, CHANGE);
}



/*****************************************************************************-
    chXIsr() Interrupt routines for radio pulses
 *****************************************************************************/
const int RC_MAX = 2150;
const int RC_MIN = 872;
const int RC_RANGE = RC_MAX - RC_MIN;
const int RC_MID = (RC_RANGE / 2) + RC_MIN ;
void ch1Isr() {
  static unsigned long riseTime = 0UL;
  unsigned long t = micros();
  if (digitalReadFast(CH1_RADIO_PIN)) riseTime = t;
  else {
    int ch1pw = t - riseTime;
    controllerX = ( 2.0 * ((float) (ch1pw - RC_MID))) / RC_RANGE;
  }
}
void ch2Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH2_RADIO_PIN)) riseTime = t;
  else  {
    int ch2pw = t - riseTime;
    controllerY = ( 2.0 * ((float) (ch2pw - RC_MID))) / RC_RANGE;
  }
}
void ch3Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH3_RADIO_PIN)) riseTime = t;
  else {
    int ch3pw = t - riseTime;
    ch3State = (ch3pw < 1500) ? false : true;
  }
}
void ch4Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH4_RADIO_PIN)) riseTime = t;
  else {
    int ch4pw = t - riseTime;
    if (ch4pw < 1200) ch4State = 0;
    else if (ch4pw < 1800) ch4State = 1;
    else ch4State = 2;
  }
}
void ch5Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH5_RADIO_PIN)) riseTime = t;
  else {
    int ch5pw = t - riseTime;
    ch5Val = ( 2.0 * ((float) (ch5pw - RC_MID))) / RC_RANGE;
  }
}
void ch6Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH6_RADIO_PIN)) riseTime = t;
  else {
    int ch6pw = t - riseTime;
    ch6Val = ( 2.0 * ((float) (ch6pw - RC_MID))) / RC_RANGE;
  }
}
