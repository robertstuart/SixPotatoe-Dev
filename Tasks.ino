
const int BATTERY_WARNING = 726;  // about 10% capacity (centivolts)
const int BATTERY_CRITICAL = 666; // about 1% cap (centivolts)

//int beepCycleCount = 0;
//boolean beepStat = false;
int *beepSequence = BEEP_OFF;
int beepPtr = 0;
boolean flip = false;
int warningCount = 0;
int criticalCount = 0;
int addFlip = 0;
boolean redLedState = false;
boolean greenLedState = false;
unsigned long taskMilliseconds = 0L;
unsigned long gravityTrigger = 0L;
//unsigned long audioTrigger = 0L;
unsigned long errorTrigger = 0L;
unsigned int taskPtr = 0;
unsigned int pingTpHCCount = 0;
unsigned long onGroundTime = 0L;
unsigned long warningTrigger = 0;
unsigned long batteryLastGood = 0;


const byte* patternBlue = BLINK_OFF;
const byte* patternYellow = BLINK_OFF;
const byte* patternRed = BLINK_OFF;
const byte* patternGreen = BLINK_OFF;
int blinkPtrBlue = 0;
int blinkPtrYellow = 0;
int blinkPtrRed = 0;
int blinkPtrGreen = 0;


/******************************************************************************
 * commonTasks()
 *****************************************************************************/
void commonTasks() {
  readUp(); 
  readXBee(); 
  blinkLed();
  checkBeep();
  battery();
  switches();
  safeAngle();
  setRunningState();
  setLedStates();
  setHcActive();
  flush();
  timeMicroseconds = micros();
  timeMilliseconds = millis();
}



/******************************************************************************
 * setRunningState()
 *
 *     Set the TP_STATE_RUNNING bit if the following are true:
 *         TP_STATE_RUN_READY is true
 *         TP_STATE_UPRIGHT is true
 *         TP_STATE_ON_GROUND or isJump is true
 *
 *      Set x and y to zero if there is no connection to
 *      a controller or STATE_MOTOR_FAULT is true.
 *
 *      Set blinking according to the above states.
 *
 **************************************************************************/
void setRunningState() {
  static boolean oldIsRunning = true;

  // Set the runnng bit to control motors
  if (isRunReady && (isUpright || isGettingUp)) {
    isRunning = true;
    if (oldIsRunning == false) {  // State change
      oldIsRunning = true;
    }
  } else {
    isRunning = false;
    if (oldIsRunning == true) { // State change
      oldIsRunning = false;
    } 
  }
}

void setLedStates() {
  byte *bluePattern;
  byte *yellowPattern;
  
  // Set the blue upboard led
  if (timeMilliseconds > (upStatTime + 200)) {
    bluePattern = BLINK_OFF;  // Timed out
    isRouteInProgress = false;
  } else {
    bluePattern = (isRouteInProgress) ? BLINK_ON : BLINK_SB;
  }

  // set yellow (state)
  if (isRouteInProgress){
    yellowPattern = (isRunning) ? BLINK_ON : BLINK_FF;
  } else if (isRunReady && isRunning) {
    setLedStates = BLINK_ON;
  } else if (isRunReady && !isRunning) {
    yellowPattern = BLINK_FF;
  } else {
    yellowPattern = BLINK_SB;
  }
      
  setBlink(LED_BU_PIN, bluePattern);
  setBlink(LED_YE_PIN, yellowPattern);
  setBlink(LED_GN_PIN, yellowPattern);
}


/******************************************************************************
 * safeAngle() Check to see if we have fallen sidways or forwards.
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



/******************************************************************************
 * battery()
 *****************************************************************************/
void battery() {
  static unsigned long batteryTrigger = 0L;
  if (timeMilliseconds > batteryTrigger) {
    batteryTrigger = timeMilliseconds + 1000;  // 1 per second
    battVolt = ((float) analogRead(BATT_PIN)) * .01592;
//    Serial.print(analogRead(BATT_PIN)); Serial.print(" "); Serial.println(battVolt);
  }
}



/******************************************************************************
 * gyroTemperature() Not used. Doesn't improve drift over small 
 *                   temperature ranges.
 ******************************************************************************/
//void gyroTemperature() {
//  static unsigned long gyroTemperatureTrigger = 0UL;
//  if (timeMilliseconds > gyroTemperatureTrigger) {
//    gyroTemperatureTrigger = timeMilliseconds + 1000;  // 1 per second
//    float f = readFahr();
//    temperatureDriftYaw = (f - baseGyroTemp) * 0.172;
//  }
//}



/******************************************************************************
 *  blink() 
 ******************************************************************************/
void blinkLed() {
  static unsigned long blinkTrigger = 0L;
  if (timeMilliseconds > blinkTrigger) {
    blinkTrigger = timeMilliseconds + 100;  // 10 per second

    // Blink the Blue
    int b = (patternBlue[blinkPtrBlue++] == 1) ? HIGH : LOW;
    if (patternBlue[blinkPtrBlue] == END_MARKER) blinkPtrBlue = 0;
    digitalWrite(LED_BU_PIN, b);

    // Blink the Green
    b = (patternGreen[blinkPtrGreen++] == 1) ? HIGH : LOW;
    if (patternGreen[blinkPtrGreen] == END_MARKER) blinkPtrGreen = 0;
    digitalWrite(LED_GN_PIN, b);

    // Blink the Yellow
    b = (patternYellow[blinkPtrYellow++] == 1) ? HIGH : LOW;
    if (patternYellow[blinkPtrYellow] == END_MARKER) blinkPtrYellow = 0;
    digitalWrite(LED_YE_PIN, b);

    // Blink route number on red
//    if (++routeOffCount >=5) {
//      routeOffCount = 0;
//      if (routeCycle <= routeTablePtr) {
//        digitalWrite(LED_RE_PIN, HIGH);
//      }
//      routeCycle++;
//      if (routeCycle >= (routeTablePtr + 3)) {
//        routeCycle = 0;
//      }
//    } else if (routeOffCount == 2) {
//      digitalWrite(LED_RE_PIN, LOW);
//    }
  }  
}



/******************************************************************************
 *  setBlink() Set blink patter for led
 ******************************************************************************/
void setBlink(int led, byte* pattern) {
  switch (led) {
    case LED_BU_PIN:  // Also blinks the green
      if (patternBlue != pattern) {
        patternBlue = pattern;
        blinkPtrBlue = 0;
        blinkPtrYellow = 0;
        blinkPtrRed = 0;
        blinkPtrGreen = 0;
      }
      break;
    case LED_YE_PIN:
      if (patternYellow != pattern) {
        patternYellow = pattern;
        blinkPtrBlue = 0;
        blinkPtrYellow = 0;
        blinkPtrRed = 0;
        blinkPtrGreen = 0;
     }
      break;
    case LED_RE_PIN:
      if (patternRed != pattern) {
        patternRed = pattern;
        blinkPtrBlue = 0;
        blinkPtrYellow = 0;
        blinkPtrRed = 0;
        blinkPtrGreen = 0;
      }
      break;
    case LED_GN_PIN:
      if (patternGreen != pattern) {
        patternGreen = pattern;
        blinkPtrBlue = 0;
        blinkPtrYellow = 0;
        blinkPtrRed = 0;
        blinkPtrGreen = 0;
      }
      break;
    default:
      break;
  }
}



void beep(int seq[]) {
  beepPtr = 0;
  beepSequence = seq;
}

void checkBeep() {
  static unsigned long beepTrigger = 0;
  if (timeMilliseconds > beepTrigger) {
    beepTrigger = timeMilliseconds + 500;
    int beepVal = beepSequence[beepPtr];
    if (beepVal == END_MARKER) {
      noTone(SPEAKER_PIN);
    } else {
      beepVal = beepSequence[beepPtr++];
      tone(SPEAKER_PIN, beepVal);
    }
  }
}



/******************************************************************************
 * switches()
 *      Toggle TP_STATE_RUN_READY on yellow switch.  1 sec dead period.
 ******************************************************************************/
void switches() {
  static unsigned int buTimer = 0;
  static boolean buState = false;
  static boolean oldBuState = false;

  static unsigned int yeTimer = 0;
  static boolean yeState = false;
  static boolean oldYeState = false;
  
  static unsigned int reTimer = 0;
  static boolean reState = false;
  static boolean oldReState = false;
  
  static unsigned int gnTimer = 0;
  static boolean gnState = false;
  static boolean oldGnState = false;
  
  // Debounce Blue
  boolean bu = digitalRead(SW_BU_PIN) == LOW;
  if (bu) buTimer = timeMilliseconds;
  if ((timeMilliseconds - buTimer) > 50) buState = false;
  else buState = true;
  
  // Debounce Yellow
  boolean ye = digitalRead(SW_YE_PIN) == LOW;
  if (ye) yeTimer = timeMilliseconds;
  if ((timeMilliseconds - yeTimer) > 50) yeState = false;
  else yeState = true;

  // Debounce Red
  boolean re = digitalRead(SW_RE_PIN) == LOW;
  if (re) reTimer = timeMilliseconds;
  if ((timeMilliseconds - reTimer) > 50) reState = false;
  else reState = true;

  // Debounce Green (back switch)
  boolean gn = digitalRead(SW_GN_PIN) == LOW;
  if (gn) gnTimer = timeMilliseconds;
  if ((timeMilliseconds - gnTimer) > 50) gnState = false;
  else gnState = true;

  // Blue press transition. Toggle route enable.
  if (buState && (!oldBuState)) {
//    sendUMsg(TOUP_RT_ENABLE, 0); // toggle
  }

  // Green press transition.  Start route.
  if (gnState && (!oldGnState)) {
//    sendUMsg(TOUP_RT_START, 0);
  }

  // Yellow press transition
  if (yeState && (!oldYeState)) {
//    if (isRouteInProgress)  sendUMsg(TOUP_RT_START, 0);
//    else isRunReady = !isRunReady;
    isRunReady = !isRunReady;
  }

  // Red press transition
  if (reState && (!oldReState)) {
//    sendUMsg(TOUP_RT_NUM, 1);  // Increment route number.
  }

  oldBuState = buState;
  oldYeState = yeState;
  oldReState = reState;
  oldGnState = gnState; 
}


void setHcActive() {
  unsigned static long hcTrigger = 0UL;
  if (timeMilliseconds > hcTrigger) {
    hcTrigger = timeMilliseconds + 100;
    isHcActive = (timeMilliseconds - lastHcTime) > 1000;
  }
}

void flush() { // at beginning, flush serial buffer.
  static unsigned long t = 0UL;
  static bool hasFired = false;
  if (hasFired) return;
  if (t == 0UL) t = timeMilliseconds;
  if (timeMilliseconds > (t + 2000)) {
    Serial.println("Flushed");
    hasFired = true;
  }
}

/******************************************************************************
 *  rangeAngle() Set angle value between -180 and +180
 ******************************************************************************/
double rangeAngle(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle <= -180.0) angle += 360.0;
  return angle;
}
