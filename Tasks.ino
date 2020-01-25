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
  static unsigned int blinkCount = 0;

  if (timeMilliseconds > trigger) {
    trigger = timeMilliseconds + 100;
    bool buState, gnState;

    blinkCount++;
    switch (currentBlink) {
      case SlowFlash:
        buState = ((blinkCount % 10) == 0);
        gnState = ((blinkCount % 10) == 1);
        break;
      case FastFlash:
        gnState = buState = ((blinkCount % 2) == 0);
        break;
      case SlowBlink:
        gnState = buState = (((blinkCount / 5) % 2) == 0);
        break;
      case On:
        buState = gnState = true;
        break;
      case Off:
      default:
        break;
    }
    digitalWrite(LED_BU_PIN, buState ? HIGH : LOW);
    digitalWrite(LED_GN_PIN, gnState ? HIGH : LOW);
  }
}
void blink13() {  // Just blink the Teensy
  static unsigned int blinkCount;
  digitalWrite(LED_PIN, ((blinkCount++ / 50) % 2) ? HIGH : LOW);
}




/*****************************************************************************-
   safeAngle() Check to see if we have fallen sidways or forwards.
      TODO make this dependent on speed
 *****************************************************************************/
void safeAngle() {
  static unsigned long tTime = 0UL; // time of last state change
  static boolean tState = false;  // Timed state. true = upright

  boolean cState = (abs(imu.maPitch) < 70.0); // Current real state
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
  static unsigned int timerBu = 0;
  static boolean buState = false;
  static boolean oldBuState = false;
  static unsigned int timerGn = 0;
  static boolean gnState = false;
  static boolean oldGnState = false;

  // Debounce blue switch 
  boolean swState = digitalRead(SW_BU_PIN) == LOW;
  if (swState) timerBu = timeMilliseconds;
  if ((timeMilliseconds - timerBu) > 50) buState = false;
  else buState = true;
  if (buState && (!oldBuState)) {  // Blue switch press transition?
    isRunReady = !isRunReady;
  }
  oldBuState = buState;

  // Debounce green switch 
  swState = digitalRead(SW_GN_PIN) == LOW;
  if (swState) timerGn = timeMilliseconds;
  if ((timeMilliseconds - timerGn) > 50) gnState = false;
  else gnState = true;
  if (gnState && (!oldGnState)) {  // Green Switch press transition?
    isRunReady = !isRunReady;
  }
  oldGnState = gnState;
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
