/*****************************************************************************-
 *                            SixPotatoe.ino
 *****************************************************************************/
#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"

// Test defines
const bool IS_TEST1 = false;  // Set to be true for the 1st system test.
const bool IS_TEST2 = false;  // Set to be true for the 2nd system test.
const bool IS_TEST3 = true;  // Set to be true for the 3nd system test.

// System constants
//const float GYRO_SENS = 0.0696;      // Multiplier to get degrees. 
const float GYRO_WEIGHT = 0.997;
const float WHEEL_DIA_INCH = 4.834;
const float WHEEL_DIA_MM = 125.0;
const float WHEEL_CIRC = M_PI * (WHEEL_DIA_INCH / 12.0);
const float WHEEL_CIRC_MM = M_PI * (WHEEL_DIA_MM / 12.0);
const float TICKS_PER_ROTATION = 329.54;
const float TICKS_PER_FOOT = TICKS_PER_ROTATION / WHEEL_CIRC;
const float TICKS_PER_METER = TICKS_PER_ROTATION / WHEEL_CIRC_MM;
const float ENC_FACTOR = 1000000.0 / TICKS_PER_FOOT;
const long ENC_FACTOR_M = (long) (ENC_FACTOR * 1000.0f);
const float FPS_TO_PW = 17.5;
const float DEAD_ZONE = 0.0;
const float MAX_FPS = 10.0;

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
float CONST_ANGLE_TO_FPS = 0.2;
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
int wMFpsRight = 0;
float wFpsRight = 0.0;
int wMFpsLeft = 0;
float wFpsLeft = 0.0;
int wMFps = 0;
float wFps = 0.0;
float targetWFpsRight = 0.0;
float targetWFpsLeft = 0.0;

// Run variables
float rotation3 = 0.0;
float cos3 = 0.0;
float lpfCos3 = 0.0;
float lpfCos3Old = 0.0;
float coFps = 0.0;
float targetCoFps = 0.0;
float targetWFps = 0.0;
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

float targetWhFpsRight = 0.0;
float targetWhFpsLeft = 0.0;
long whMFpsRight = 0;
long whMFpsLeft = 0;
float whFpsRight = 0.0;
float whFpsLeft = 0.0;
float whFps = 0.0;

// RC variables
volatile float controllerX = 0.0;   // ch1 steering
volatile float controllerY = 0.0;   // ch2 accelerator
volatile boolean ch3State = false;  // ch3 toggle
volatile int ch4State = 0;          // ch4 3-position switch
volatile float ch5Val = 0.0;        // ch5 top left potentiometer
volatile float ch6Val = 0.0;        // ch6 top right potentiometer.

char message[200] = "";


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
// Check motor control
void systemTest3() {
  static boolean toggle = false;
  static int m = 0;

  while (true) {
    commonTasks();
    if (isNewImuData()) {
      int x = (int) (controllerX * 20.0);
      int y = (int) (controllerY * 255.0);
      int r = y + x;
      int l = y - x;
      setMotorRight(abs(r), r > 0);
      setMotorLeft(abs(l), l > 0);
      readSpeedRight();
      readSpeedLeft();
//      sprintf(message, "%7.2f %7.2f %5d %5d %5d %5d %5d", wFpsRight, wFpsLeft, x, y, r, l, isRunning);
//      Serial.println(message);
    }
  }
}
