/*****************************************************************************-
 *                            SixPotatoe.ino
 *****************************************************************************/
#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"

// Test defines
const bool IS_TEST1 = false;  // Set to be true for the 1st system test.
const bool IS_TEST2 = false;  // Set to be true for the 2nd system test.
const bool IS_TEST3 = false;  // Set to be true for the 3nd system test.
const bool IS_TEST4 = false;  // Set to be true for the 4th system test.

// System constants
const float GYRO_WEIGHT = 0.997;
const float WHEEL_DIA_MM = 125.0;
const float TICKS_PER_ROTATION = 329.5;
const float USEC_TO_KPH = (3600 * WHEEL_DIA_MM * M_PI) / TICKS_PER_ROTATION;
const float KPH_TO_PW = 8.1;
const float MAX_KPH = 22.0;  // Maximum target for controller

/*****************************************************************************-
 *  Pin definitions
 *****************************************************************************/
const int PWM_LEFT_PIN    = 14;
const int DIR_LEFT_PIN    = 16;
const int PWM_RIGHT_PIN   = 15;
const int DIR_RIGHT_PIN   = 17;

const int ENC_A_LEFT_PIN  = 20;
const int ENC_B_LEFT_PIN  = 22;
const int ENC_A_RIGHT_PIN = 21;
const int ENC_B_RIGHT_PIN = 23;

const int CH1_RADIO_PIN   =  2;
const int CH2_RADIO_PIN   =  3;
const int CH3_RADIO_PIN   =  4;
const int CH4_RADIO_PIN   =  5;
const int CH5_RADIO_PIN   =  6;
const int CH6_RADIO_PIN   =  7;

const int LED_PIN         = 13;
const int LED_A_PIN       = 12;

const int SW_A_PIN        =  9;

// Tunable variables
float CONST_COS_ROTATION = 4.5;
float CONST_COS_LPF = 0.98;
float valSetV = 2.7;
float CONST_ACCEL_LPF = 0.95;  
//float CONST_ACCEL_PITCH = x.x;   // Cancel out acceleration from gPitch
float CONST_ERROR_TO_ANGLE = 2.0;
float CONST_ANGLE_TO_KPH = 0.2;
float ACCEL_PITCH_OFFSET = 1.4;   // pitch offset
float MOTOR_GAIN = 3.0;

enum BlinkState {
  Off,
  SlowFlash,
  FastFlash,
  SlowBlink,
  On
};

BlinkState currentBlink = Off;

// Imu variables
float gyroPitchDelta = 0.0;
float gaPitch = 0.0;
float gaRoll = 0.0;
float gYaw = 0.0;
float maPitch = 0.0;

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
float targetWKphLeft = 0.0;

// Run variables
float rotation3 = 0.0;
float cos3 = 0.0;
float lpfCos3 = 0.0;
float lpfCos3Old = 0.0;
float coKph = 0.0;
float targetCoKph = 0.0;
float targetWKph = 0.0;
bool isGettingUp = false;
unsigned long gettingUpStartTime = 0;

unsigned long timeMilliseconds = 0UL;
unsigned long timeMicroseconds = 0UL;
bool isRunning = false;
bool isRunReady = false;
bool isUpright = false;
bool isRcActive = true;  // Set this false if signal disappears.

//int bCount = 0;
unsigned long upStatTime = 0UL;

long tickPosition = 0L;
double coTickPosition = 0.0;


volatile int writeCount = 0;

// RC variables
volatile float controllerX = 0.0;   // ch1 steering
volatile float controllerY = 0.0;   // ch2 accelerator
volatile boolean ch3State = false;  // ch3 toggle
volatile int ch4State = 0;          // ch4 3-position switch
volatile float ch5Val = 0.0;        // ch5 top left potentiometer
volatile float ch6Val = 0.0;        // ch6 top right potentiometer.

char message[200] = "";
#define DBUFF_SIZE 10000
int dBuffPtr = 0;
boolean isDBuffFull = false;
String dBuff[DBUFF_SIZE];


/*****************************************************************************_
 * setup()
 *****************************************************************************/
void setup() {
  Serial.begin(115200);
  Wire.begin();
 
  // Motor pins are initialized in motorInit()
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_A_PIN, OUTPUT);
  pinMode(SW_A_PIN, INPUT_PULLUP);
    
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(LED_A_PIN, LOW);

  rcInit(); 
  imuInit();
  motorInit();
  delay(100); // For switches?

  
////  delay(10);
//  OpenLog myLog; //Create instance
////  delay(10);
//  myLog.begin(); //Open connection to OpenLog (no pun intended)
////  delay(10);
//  Serial.println(myLog.getVersion());
//  Serial.println("OpenLog Write File Test2");
//  myLog.println("This2 goes to the log file2");
////  delay(10);
//  Serial.println("This goes to the terminal2");
//  float batteryVoltage = 3.4;
//  myLog.println("Batt voltage2: " + String(batteryVoltage));
////  delay(10);
//  batteryVoltage = batteryVoltage + 0.71;
//  myLog.println("Batt voltage2: " + String(batteryVoltage));
////  delay(10);
//  myLog.syncFile();
//  Serial.println(F("Done2!"));


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
  static boolean toggle = false;
  static int m = 0;
  if (isNewImuData()) {
    unsigned long newT = millis();
    sprintf(message, "%2d ms   %7.2f degrees", newT - lastT, maPitch);
    Serial.println(message);
    lastT = newT;
    if ((m++ % 10) == 0) digitalWrite(LED_PIN, (toggle = !toggle) ? HIGH : LOW);
  }
}
// Check the RC controller
void systemTest2() {
  static boolean toggle = false;
  static int m = 0;
  if (isNewImuData()) {
    if ((m++ % 10) == 0) { 
      digitalWrite(LED_PIN, (toggle = !toggle) ? HIGH : LOW);
      sprintf(message, "%5.2f %5.2f %5d %5d %5.2f %5.2f", controllerX, controllerY, ch3State, ch4State, ch5Val, ch6Val);
      Serial.println(message);
    }
  }
}
// Check motor controlers
void systemTest3() {
  while (true) {
    commonTasks();
    if (isNewImuData()) {
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
    if (isNewImuData()) {
      float x = (controllerX * 5.0);
      float y = (controllerY * 10.0);
      targetWKphRight = y + x;
      targetWKphLeft = y - x;
      checkMotors();
      sprintf(message, "%7.2f %7.2f %7.2f %7.2f %5d", wKphRight, wKphLeft, x, y, isRunning);
      Serial.println(message);
    }
  }
}
