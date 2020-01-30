/*****************************************************************************-
 *                            SixPotatoe.ino
 *****************************************************************************/
#include <vector>
#include <Wire.h>
#include "IMU.h"

// Test defines
const bool IS_TEST1 = false;  // Set to be true for the 1st system test.
const bool IS_TEST2 = false;  // Set to be true for the 2nd system test.
const bool IS_TEST3 = false;  // Set to be true for the 3nd system test.
const bool IS_TEST4 = false;  // Set to be true for the 4th system test.

// System constants
//const float GYRO_WEIGHT = 0.997;
const float WHEEL_DIA_MM = 125.0;
const float TICKS_PER_ROTATION = 329.5;
const float USEC_TO_KPH = (3600 * WHEEL_DIA_MM * M_PI) / TICKS_PER_ROTATION;
const float KPH_TO_PW = 8.1;
const float MAX_KPH = 22.0;  // Maximum target for controller

/*****************************************************************************-
 *  Pin definitions
 *****************************************************************************/
const int PWM_LEFT_PIN    = 14;
const int PWM_RIGHT_PIN   = 15;
const int DIR_LEFT_PIN    = 16;
const int DIR_RIGHT_PIN   = 17;

const int ENC_A_LEFT_PIN  = 20;
const int ENC_B_LEFT_PIN  = 21;
const int ENC_A_RIGHT_PIN = 22;
const int ENC_B_RIGHT_PIN = 23;

const int CH1_RADIO_PIN   =  2;
const int CH2_RADIO_PIN   =  3;
const int CH3_RADIO_PIN   =  4;
const int CH4_RADIO_PIN   =  5;
const int CH5_RADIO_PIN   =  6;
const int CH6_RADIO_PIN   =  7;

const int LED_PIN         = 13;
const int LED_GN_PIN      = 12;
const int LED_BU_PIN      = 11;

const int SW_BU_PIN       = 10;
const int SW_GN_PIN       =  9;

// Tunable variables
float CONST_COKPH_ROTATION = 2.1;
float CONST_COKPH_TC = 0.05; // 1.0 passes all hf, near zero passes only low freq.
float valSetV = 2.7;
float CONST_ACCEL_LPF = 0.95;  
//float CONST_ACCEL_PITCH = x.x;   // Cancel out acceleration from gPitch
float CONST_ERROR_TO_ANGLE = 2.0;
float CONST_ANGLE_TO_KPH = 0.2;
float ACCEL_PITCH_OFFSET = 1.4;   // pitch offset
float MOTOR_GAIN = 3.0;
float K12 = 50.0;   // +- constraint on target pitch
float K13 = 35.0;   // +- constraint on pitch error to prevent too rapid righting
int   K14 = 500;    // ms to get back up again after fall
float K15 = 70;     // pitch beyond which is considered to not be upright
int   K16 = 500;    // ms after fall to stay down

enum BlinkState {
  Off,
  SlowFlash,
  FastFlash,
  SlowBlink,
  On
};

BlinkState currentBlink = Off;

#define N_LOGS 15000  // 15k near max before running out of memory
float logFloats[4][N_LOGS];
int logCount = 0;
boolean isLogWrap = false;
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
// Run variables
float rotation = 0.0;
float coKph = 0.0;
float coKphOld = 0.0;
float targetCoKph = 0.0;
float targetWKph = 0.0;
boolean isStartGetUp = false;
boolean isGettingUp = false;

unsigned long timeMilliseconds = 0UL;
unsigned long timeMicroseconds = 0UL;
bool isRunning = false;
bool isRunReady = false;
bool isUpright = false;

//int bCount = 0;
//unsigned long upStatTime = 0UL;

long tickPosition = 0L;
double coTickPosition = 0.0;


volatile int writeCount = 0;

// RC variables
volatile float controllerX = 0.0;   // ch1 steering
volatile float controllerY = 0.0;   // ch2 accelerator
volatile boolean ch3State = false;  // ch3 toggle
volatile int ch4State = 0;          // ch4 3-position switch
volatile float ch5Val = 0.0;        // ch5 top left potentiometer
volatile int ch5State = 0;
volatile float ch6Val = 0.0;        // ch6 top right potentiometer.

char message[200] = "";
#define DBUFF_SIZE 10000
int dBuffPtr = 0;
boolean isDBuffFull = false;
String dBuff[DBUFF_SIZE];

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
  pinMode(LED_GN_PIN, OUTPUT);
  pinMode(SW_BU_PIN, INPUT_PULLUP);
  pinMode(SW_GN_PIN, INPUT_PULLUP);
    
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(LED_BU_PIN, LOW);
  digitalWrite(LED_GN_PIN, LOW);

  rcInit(); 
  imu.imuInit(&Serial, Wire, 1);
  motorInit();
  delay(100); // For switches?

  
//  for (int i = 0; i < 200; i++) {
//    sprintf(message, "%3d,%6.2f,%7.3f", i, sin((float) i), cos((float) i)); 
//    log();
//  }
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
    sprintf(message, "%2d ms   %7.2f degrees", ((int) (newT - lastT)), imu.maPitch);
    Serial.println(message);
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
      sprintf(message, 
              "ch1:%5.2f     ch2:%5.2f     ch3:%1d     ch4:%1d     ch5:%5.2f      ch6:%5.2f", 
              controllerX, controllerY, ch3State, ch4State, ch5Val, ch6Val);
      Serial.println(message);
    }
    blink13();
  }
}
// Check motor controlers
void systemTest3() {
  while (true) {
    commonTasks();
    if (imu.isNewImuData()) {
      int x = (int) (controllerX * 20.0);
      int y = (int) (controllerY * 100.0);
      int r = y + x;
      int l = y - x;
      setMotorRight(abs(r), r > 0);
      setMotorLeft(abs(l), l > 0);
      readSpeedRight();
      readSpeedLeft();
      sprintf(message, "%7.2f %7.2f %5d %5d %5d", wKphRight, wKphLeft, r, l, isRunning);
      Serial.println(message);
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
      sprintf(message, "%7.2f %7.2f %7.2f %7.2f %5d", wKphRight, wKphLeft, x, y, isRunning);
      Serial.println(message);
    }
  }
}
