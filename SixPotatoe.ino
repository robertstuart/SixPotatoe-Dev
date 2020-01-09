
const bool IS_TEST1 = false;  // Set to be true for the 1st system test.
const bool IS_TEST2 = true;  // Set to be true for the 2nd system test.

#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"

//const float GYRO_SENS = 0.0696;      // Multiplier to get degrees. 
const float GYRO_WEIGHT = 0.997;
const float WHEEL_DIA_INCH = 4.834;
const float WHEEL_CIRC = M_PI * (WHEEL_DIA_INCH / 12.0);
const float TICKS_PER_ROTATION = 329.54;
const float TICKS_PER_FOOT = TICKS_PER_ROTATION / WHEEL_CIRC;
const float ENC_FACTOR = 1000000.0 / TICKS_PER_FOOT;
const long ENC_FACTOR_M = (long) (ENC_FACTOR * 1000.0f);
const float FPS_TO_PW = 17.5;
const float DEAD_ZONE = 0.0;
const float MAX_FPS = 10.0;

#define PWM_LEFT_PIN     14
#define DIR_LEFT_PIN     16
#define PWM_RIGHT_PIN    15
#define DIR_RIGHT_PIN    17

#define ENC_A_LEFT_PIN   20
#define ENC_B_LEFT_PIN   22
#define ENC_A_RIGHT_PIN  21
#define ENC_B_RIGHT_PIN  23

#define CH1_RADIO_PIN     2
#define CH2_RADIO_PIN     3
#define CH3_RADIO_PIN     4
#define CH4_RADIO_PIN     5
#define CH5_RADIO_PIN     6
#define CH6_RADIO_PIN     7

#define LED_PIN          13
#define LED_A_PIN        11

#define SW_A_PIN        10

// Constants to be set
float CONST_COS_ROTATION = 4.5;
float CONST_COS_LPF = 0.98;
float valSetV = 2.7;
float CONST_ACCEL_LPF = 0.95;  
//float CONST_ACCEL_PITCH = x.x;   // Cancel out acceleration from gPitch
float CONST_ERROR_TO_ANGLE = 2.0;
float CONST_ANGLE_TO_FPS = 0.2;
float ACCEL_PITCH_OFFSET = 1.4;   // pitch offset
float MOTOR_GAIN = 3.0;


// Blink sequences .1 sec sequences
const byte END_MARKER = 42;
byte BLINK_OFF[] = {0,END_MARKER};               // Off
byte BLINK_SF[] = {1,0,0,0,0,0,0,0,END_MARKER};  // Slow flash
byte BLINK_FF[] = {1,0,END_MARKER};              // Fast flash
byte BLINK_SB[] = {1,1,1,1,0,0,0,0,END_MARKER};  // Slow blink
byte BLINK_ON[] = {1,END_MARKER};                // On

// Tone sequences.  .25 sec sequences
int BEEP_UP[] = {750,900,END_MARKER};
int BEEP_OFF[] = {END_MARKER};

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
bool isMotorDisable = false;
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
volatile int ch1pw = 0; 
volatile int ch2pw = 0; 
volatile int ch3pw = 0; 
volatile int ch4pw = 0; 
volatile int ch5pw = 0; 
volatile int ch6pw = 0; 
volatile float controllerX = 0.0; // ch1
volatile float controllerY = 0.0; // ch2
volatile boolean ch3State = false;
volatile int ch4State = 0;
volatile float ch5Val = 0.0;
volatile float ch6Val = 0.0;

#define MSG_SIZE 100
char message[MSG_SIZE] = "";

/*******************************************************************************
 * setup()
 ******************************************************************************/
void setup() {
  Serial.begin(115200);
  Wire.begin();
//  Wire.setClock(400000);
  // Motor pins are initialized in motorInit()
  pinMode(LED_PIN, OUTPUT);
    
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(LED_A_PIN, LOW);

  rcInit(); 
  imuInit();
  motorInit();

  
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
  else run();

}



/*****************************************************************************-
 * systemTest?()  
 *           TODO Explain test
 *****************************************************************************/
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
