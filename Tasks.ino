
unsigned int ledOnTime = 0UL;
unsigned int ledOffTime = 0UL;
unsigned int ledAOnTime = 0UL;
unsigned int ledAOffTime = 0UL;

/*****************************************************************************-
 * commonTasks()
 *****************************************************************************/
void commonTasks() {
  blinkLed();
  switches();
  safeAngle();
  setRunningState();
  blinkLed();
  timeMicroseconds = micros();
  timeMilliseconds = millis();
}



/*****************************************************************************-
 * setRunningState()
 *****************************************************************************/
void setRunningState() {
//  static boolean oldIsRunning = true;

  // Set isRunning variable to control motors
  if (isRunReady && (isUpright || isGettingUp)) {
    isRunning = true;
//    if (oldIsRunning == false) {  // State change
//      oldIsRunning = true;
//    }
  } else {
    isRunning = false;
//    if (oldIsRunning == true) { // State change
//      oldIsRunning = false;
//    } 
  }
}



/*****************************************************************************-
 * setBlink()  Set the blink pattern for an LED
 *****************************************************************************/
void setBlink(int pin, int onTime, int offTime) {
  if (pin == LED_PIN) {
    ledOnTime = onTime;
    ledOffTime = offTime;
  }
  if (pin == LED_A_PIN) {
    ledAOnTime = onTime;
    ledAOffTime = offTime;
  }
}



/*****************************************************************************-
 * blinkLed()
 *****************************************************************************/
void blinkLed() {
  static bool isLedOn = false;
  static unsigned long ledTrigger = 0;
  if (timeMilliseconds > ledTrigger) {
    digitalWrite(LED_PIN, isLedOn ? LOW : HIGH);
    ledTrigger = timeMilliseconds + (isLedOn ? ledOffTime : ledOnTime);
    isLedOn = !isLedOn;
  }
  static bool isLedAOn = false;
  static unsigned long ledATrigger = 0;
  if (timeMilliseconds > ledATrigger) {
    digitalWrite(LED_A_PIN, isLedAOn ? LOW : HIGH);
    ledATrigger = timeMilliseconds + (isLedAOn ? ledAOffTime : ledAOnTime);
    isLedAOn = !isLedAOn;
  }
}



/*****************************************************************************-
 * safeAngle() Check to see if we have fallen sidways or forwards.
 *    TODO make this dependent on speed
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
 * switches()
 *      Check switches and debounce
 ******************************************************************************/
void switches() {
  static unsigned int timerA = 0;
  static boolean aState = false;
  static boolean oldAState = false;
    
  // Debounce A switch
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
 *  rcRadioInit() 
 *****************************************************************************/
void rcRadioInit() {
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
 *  chXIsr() Interrupt routines for radio pulses
 *****************************************************************************/
void ch1Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH1_RADIO_PIN)) riseTime = t;
  else ch1pw = t - riseTime; 
}
void ch2Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH2_RADIO_PIN)) riseTime = t;
  else ch2pw = t - riseTime; 
}
void ch3Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH3_RADIO_PIN)) riseTime = t;
  else ch3pw = t - riseTime; 
}
void ch4Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH2_RADIO_PIN)) riseTime = t;
  else ch4pw = t - riseTime; 
  lastRcPulse = t;
}
void ch5Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH5_RADIO_PIN)) riseTime = t;
  else ch5pw = t - riseTime; 
}
void ch6Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH6_RADIO_PIN)) riseTime = t;
  else ch6pw = t - riseTime; 
}



/*****************************************************************************-
 *  readRcRadio() 
 *****************************************************************************/
void readRcRadio() {
  float p;
  if (timeMicroseconds > (lastRcPulse + 500000)) { // 1/2 sec without something from RC?
    controllerX = 0.0;
    controllerY = 0.0;
    ch3sw = false;
  } else {
    p = ((float) (ch2pw - 1500));
    controllerY = p / 326.0;
    p = ((float) (ch4pw - 1500));
    controllerX = p / 376.0;
    ch3sw = ch3pw < 1100;
    controllerY = constrain(controllerY, -1.0, 1.0);
    controllerX = constrain(controllerX, -1.0, 1.0);
  }
}
