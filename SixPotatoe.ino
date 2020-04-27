/*****************************************************************************-
 *                            SixPotatoe.ino
 *****************************************************************************/
#include <Wire.h>
#include "IMU.h"
#include "Routes.h"

// Test defines
const bool IS_TEST1 = false;  // Set to be true for the 1st system test.
const bool IS_TEST2 = false;  // Set to be true for the 2nd system test.
const bool IS_TEST3 = false;  // Set to be true for the 3nd system test.
const bool IS_TEST4 = false;  // Set to be true for the 4th system test.

// wheel and motor constants
//const float GYRO_WEIGHT = 0.997;
const float WHEEL_DIA_MM = 120.6;
//const float TICKS_PER_ROTATION = 329.5;  // 612 RPM motor
const float TICKS_PER_ROTATION = 460.8;  // 437 RPM motor
const float TICKS_PER_METER = (1000.0 / (M_PI * WHEEL_DIA_MM)) * TICKS_PER_ROTATION;
const float USEC_TO_KPH = (3600 * WHEEL_DIA_MM * M_PI) / TICKS_PER_ROTATION;
//const float KPH_TO_PW = 8.1;  // 612 RPM motor
const float KPH_TO_PW = 12.4;  // 437 RPM motor

/*****************************************************************************-
 *  Pin definitions
 *****************************************************************************/
const int PWM_LEFT_PIN    = 14;
const int PWM_RIGHT_PIN   = 15;
const int DIR_LEFT_PIN    = 16;
const int DIR_RIGHT_PIN   = 17;

const int ENC_B_LEFT_PIN  = 20;
const int ENC_A_LEFT_PIN  = 21;
const int ENC_B_RIGHT_PIN = 22;
const int ENC_A_RIGHT_PIN = 23;

const int CH1_RADIO_PIN   =  7;
const int CH2_RADIO_PIN   =  6;
const int CH3_RADIO_PIN   =  5;
const int CH4_RADIO_PIN   =  4;
const int CH5_RADIO_PIN   =  3;
const int CH6_RADIO_PIN   =  2;

const int LED_PIN         = 13;
const int LED_BU_PIN      = 12;
const int SW_BU_PIN       = 11;
const int WATCHDOG_PIN    =  8;

// Tunable variables
const float K0  = 4.0;      // Motor gain
const float K1  = 1.6;
const float K2  = 0.05;     // 1.0 passes all hf, near zero passes only low freq.
const float K3 = 0.213;     // Accelerometer to Kph
const float K5  = 2.0;      // Speed error to angle
const float K8  = 0.2;      // bowl roll compensation at top.
const float K10 = 1.4;      // accelerometer pitch offset
const float K12 = 50.0;     // +- constraint on target pitch
//float K13 = 30.0;           // +- constraint on pitch error to prevent too rapid righting
const float K13 = 20.0;     // +- constraint on pitch error to prevent too rapid righting
const float K14 = 0.15;     // Angle error to Kph
const float K15 = 70;       // pitch beyond which is considered to not be upright
const int   K16 = 50;       // ms time for pitch < K16 to be not upright
const float K17 = 0.1;      // "D"
const int   K20 = 80;       // LED brightness, 0-255;
const float K21 = 0.95;     // TC for accelCoKph
//const float K30 = 22.0;     // Maximum Kph target for controller, 612 RPM
const float K30 = 16.0;     // Maximum Kph target for controller, 437 RPM
const float K31 = 5.0;      // Maximum speed for Kph on ground.
const float K32 = 2.0;      // Maximum sterring on ground.

enum BlinkState {
  BLINK_OFF,        //  motors off, no route
  BLINK_ON,         //  motors on,  no route 
  BLINK_SLOW_FLASH, //  motors off, running route
  BLINK_SLOW,       //  motors on,  running route
  BLINK_FAST_FLASH  //  motors on,  fallen
};

BlinkState currentBlink = BLINK_OFF;

#define N_FLOAT_LOGS 15000  // 15k near max before running out of memory
#define N_STR_LOGS 100  // 15k near max before running out of memory
float logFloats[4][N_FLOAT_LOGS];
String logStrs[N_STR_LOGS];
int logFloatCount = 0;
boolean isLogFloatWrap = false;
int logStrCount = 0;
boolean isLogStrWrap = false;
String logHeader = "No Header";

// Motor varialbles
volatile long tickPositionRight = 0;
volatile long tickPositionLeft = 0;
volatile long tickTimeRight = 0;
volatile long tickTimeLeft = 0;
volatile long tickSumRight = 0;
volatile long tickSumLeft = 0;
volatile long tickCountRight = 0;
volatile long tickCountLeft = 0;
float wKphRight = 0.0;
float wKphLeft = 0.0;
float wKph = 0.0;
float targetWKphRight = 0.0;
float motorTargetKphRight = 0.0;
float targetWKphLeft = 0.0;
float motorTargetKphLeft = 0.0;
int motorRightPw = 0;
int motorLeftPw = 0;

// Run variables
float coKph = 0.0;
float balanceTargetKph = 0.0;
float balanceSteerAdjustment = 0.0;
float targetWKph = 0.0;

unsigned long timeMilliseconds = 0UL;
unsigned long timeMicroseconds = 0UL;
bool isRunning = false;
bool isBowlBalancing = false;
bool isAir = false;
bool isUpright = false;
bool isZeroG = false;

long tickPosition = 0L;
double tickMeters = 0.0;
double coTickPosition = 0.0;


volatile int writeCount = 0;

// RC variables
volatile float controllerX = 0.0;   // ch1 steering
volatile float controllerY = 0.0;   // ch2 accelerator
volatile boolean ch3State = false;  // ch3 toggle
volatile int ch4State = 0;          // ch4 3-position switch
volatile float ch5Val = 0.0;        // ch5 top left potentiometer
volatile int ch5State = 0;          // ch5 states 1-5
volatile float ch6Val = 0.0;        // ch6 top right potentiometer.

#define DBUFF_SIZE 10000
int dBuffPtr = 0;
boolean isDBuffFull = false;
String dBuff[DBUFF_SIZE];

// Nav public variables
struct loc {
  double x;
  double y;
};
struct loc currentLoc;
struct loc targetLoc;
struct loc pivotLoc;
struct loc hugStartLoc;
struct loc coSetLoc;
float zeroGKph = 0.0;
unsigned long timeRun = 0;
unsigned long timeStart = 0;
char routeCurrentAction = 0;
int routeStepPtr = 0;
boolean isStartReceived = false;
boolean isRouteInProgress = false;
float routeKph = 0.0;
String routeTitle = "";
float turnRadius = 0.0;
float bowlEndDistance = 0.0;
float bowlArray[100];
unsigned long bowlStartTime = 0UL;
int bowlArraySize = 0;
float bowlTargetPitch = 0.0;
//int rampTicksRight = 0;
//int rampTicksLeft = 0;
float bowlCompleted = 0.0;
boolean isLoadedRouteValid = true;
String *currentRoute = go;
float getupSpeed = 0.0;
const float STEP_ERROR = -42.42;
int originalStepStringPtr = 0;
String stepString = "";
int numLen = 0;
double dDiff = 0.0D;
float targetBearing = 0.0;
float targetDistance = 0.0;
float targetRunDistance = 0.0;
float startTurnBearing = 0.0;
float currentRotation = 0.0;
float stepRotation = 0.0;
double stepDistance = 0.0D;
double currentDistance = 0.0D;
double startOrientation = 0.0;
struct loc startLoc;
bool isGettingUp = false;
int getUpTime = 0;
unsigned long getUpEndTime = 0UL;
float getUpSpeed = 0.0;
float bowlWKph = 0.0;
float airWKph = 0.0;
unsigned long airStartTime = 0UL;

IMU imu;

/*****************************************************************************_
 * setup()
 *****************************************************************************/
void setup() {
  Serial.begin(115200);
  Wire.begin();
 
  // Motor pins are initialized in motorInit()
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_BU_PIN, OUTPUT);
  pinMode(WATCHDOG_PIN, OUTPUT);
  pinMode(SW_BU_PIN, INPUT_PULLUP);
    
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(WATCHDOG_PIN, LOW);
  digitalWrite(LED_BU_PIN, LOW);

  rcInit(); 
  imu.imuInit(Wire, 1);
  motorInit();
  delay(100); // For switches?
}



/*******************************************************************************
 * loop()
 ******************************************************************************/
void loop() {
  if (IS_TEST1) systemTest1();
  else if (IS_TEST2)  systemTest2();
  else if (IS_TEST3)  systemTest3();
  else if (IS_TEST4)  systemTest4();
  else run();
}



/*****************************************************************************-
 * systemTest?()  
 *           TODO Explain test
 *****************************************************************************/
// Check IMU
void systemTest1() {
  static unsigned long lastT = 0;
  if (imu.isNewImuData()) {
    unsigned long newT = millis();
    Serial.printf(
            "Pitch:%6.2f   Roll:%6.2f   Yaw:%6.2f   Period:%2d ms\n", 
            imu.maPitch, imu.maRoll, imu.maYaw, ((int) (newT - lastT)));
    lastT = newT;
    blink13();
  }
}
// Check the RC controller and battery power
void systemTest2() {
  static boolean toggle = false;
  static int m = 0;
  if (imu.isNewImuData()) {
    if ((m++ % 10) == 0) { 
      digitalWrite(LED_PIN, (toggle = !toggle) ? HIGH : LOW);
      Serial.printf("ch1:%5.2f     ch2:%5.2f     ch3:%1d     ch4:%1d     ch5:%5.2f   ch5St5ate:%1d    ch6:%5.2f\n", 
                    controllerX, controllerY, ch3State, ch4State, ch5Val, ch5State, ch6Val);
    }
    blink13();
  }
}
// Check motor controlers and encoders
void systemTest3() {
  while (true) {
    commonTasks();
    if (imu.isNewImuData()) {
      int x = (int) (controllerX * 20.0);
//      int y = (int) (controllerY * 100.0);
      int y = (int) (controllerY * 255.0);
      int r = y + x;
      int l = y - x;
      setMotorRight(abs(r), r > 0);
      setMotorLeft(abs(l), l > 0);
      readSpeedRight();
      readSpeedLeft();
      Serial.printf("%7.2f %7.2f %5d %5d %5d\n", wKphRight, wKphLeft, r, l, isRunning);
      blink13();
    }
  }
}
// Check speed control
void systemTest4() {
  while (true) {
    commonTasks();
    if (imu.isNewImuData()) {
      float x = (controllerX * 5.0);
      float y = (controllerY * 10.0);
      targetWKphRight = y + x;
      targetWKphLeft = y - x;
      runMotors();
      Serial.printf("%7.2f %7.2f %7.2f %7.2f %5d\n", wKphRight, wKphLeft, x, y, isRunning);
      blink13();
    }
  }
}
