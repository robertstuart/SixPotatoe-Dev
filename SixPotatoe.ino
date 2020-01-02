
const bool IS_SYSTEM_TEST = false;  // Set to be true to test the system.

#include <Wire.h>
#include <LSM6.h>


const float GYRO_SENS = 0.0696;      // Multiplier to get degrees. 
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

LSM6 lsm6;

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
float controllerX = 0.0;
float controllerY = 0.0;
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
volatile unsigned long ch1pw = 0UL; 
volatile unsigned long ch2pw = 0UL; 
volatile unsigned long ch3pw = 0UL; 
volatile unsigned long ch4pw = 0UL; 
volatile unsigned long ch5pw = 0UL; 
volatile unsigned long ch6pw = 0UL; 
volatile unsigned long lastRcPulse = 0UL;
boolean ch3sw = false;

#define MSG_SIZE 100
char message[MSG_SIZE] = "";

/*******************************************************************************
 * setup()
 ******************************************************************************/
void setup() {
  Serial.begin(115200);
  // Motor pins are initialized in motorInit()
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_A_PIN, OUTPUT);
  pinMode(SW_A_PIN, INPUT_PULLUP);
    
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(LED_A_PIN, LOW);

  imuInit();
  motorInit();
}



/*******************************************************************************
 * loop()
 ******************************************************************************/
void loop() {
  if (IS_SYSTEM_TEST)  systemTest();
  else run();
}



/*****************************************************************************-
 * systemTest()  
 *     The following code runs when the IS_SYSTEM_TEST variable at the top of 
 *     this file is set to "true".  If everything is working, the speed of the
 *     motors will be conrolled by 
 *          a) the steering on the RC control, 
 *          b) the accelerator on the RC control and 
 *          c) the tilt of SixPotatoe.  
 *      To run this test, 
 *          a) set the IS_SYSTEM_TEST variable to true,
 *          b) compile an load the code,
 *          c) turn on the main power switch on SixPotatoe
 *          d) press the run on SixPotatoe
 *****************************************************************************/
void systemTest() {
//  unsigned long trigger = 0;
//  isRunning = true;
//  isMotorDisable = false;
//
//  while(true) {
//    isRunning = isRunReady;
//    if (millis() > trigger) {
//      trigger = millis() + 200;
////  Serial.print(wFpsLeft); Serial.print("\t");Serial.print(wFpsRight); Serial.print("\t");Serial.println(wFps);
//      int pw = abs(joyY);
//      Serial.print(isRunning); Serial.print(isMotorDisable); Serial.print(" ");
//      Serial.print(pw); Serial.print("\t"); Serial.println(joyY);
//      setMotorRight(pw, joyY > 0);
//      setMotorLeft(pw, joyY > 0);
//    }
//  }
}
