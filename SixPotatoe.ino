#include <Wire.h>
#include <LSM6.h>
#include "Ma_HC.h"
#include "TeensyUp.h"

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

#define XBEE_SER Serial1

#define BATT_PIN        A0 // pin 14
#define SPEAKER_PIN     30

#define PWM_LEFT_PIN     3
#define DIR_LEFT_PIN     4
#define PWM_RIGHT_PIN    5
#define DIR_RIGHT_PIN    6

#define ENC_A_LEFT_PIN   7
#define ENC_B_LEFT_PIN   8
#define ENC_A_RIGHT_PIN  9
#define ENC_B_RIGHT_PIN 10

#define IMU_INT1_PIN    20
#define IMU_INT2_PIN    21

#define LED_PIN         13
#define LED_GN_PIN      26
#define LED_BU_PIN      29
#define LED_YE_PIN      28
#define LED_RE_PIN      27

#define SW_GN_PIN       39
#define SW_BU_PIN       36
#define SW_YE_PIN       37
#define SW_RE_PIN       38

#define ACCEL_INTR_PIN  21
#define GYRO_INTR_PIN   20                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 

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
float timeDriftPitch = 0.0;  // set
float timeDriftRoll = 0.0;   // set
float timeDriftYaw = 0.0;    // set
float gyroPitchRaw = 0.0;
float gyroPitchRate = 0.0;
float gyroPitchDelta = 0.0;
float gPitch = 0.0;
float gaPitch = 0.0;
float gyroRollRaw = 0.0;
float gyroRollRate = 0.0;
float gRoll = 0.0;
float gaRoll = 0.0;
float gyroYawRaw = 0.0;
float gyroYawRate = 0.0;
float gYaw = 0.0;
float gHeading = 0.0;
float gcYaw = 0.0;
float gcHeading = 0.0;
float lpfCos3Accel = 0.0;
float aPitch = 0.0;
float lpfAPitch = 0.0;
float yAccel = 0.0;
float aRoll = 0.0;

// Motor varialbles
volatile long tickPositionRight = 0;
volatile long tickPositionLeft = 0;
volatile long tickTimeRight = 0;
volatile long tickTimeLeft = 0;
volatile long tickSumRight = 0;
volatile long tickSumLeft = 0;
volatile long tickCountRight = 0;
volatile long tickCountLeft = 0;
//volatile int intrMFpsRightSum = 0;
//volatile int intrMFpsRightCount = 0;
//volatile int intrMFpsLeftSum = 0;
//volatile int intrMFpsLeftCount = 0;
int wMFpsRight = 0;
float wFpsRight = 0.0;
int wMFpsLeft = 0;
float wFpsLeft = 0.0;
int wMFps = 0;
float wFps = 0.0;
float targetWFpsRight = 0.0;
float targetWFpsLeft = 0.0;

// Run variables
float joyX = 0.0;
float joyY = 0.0;
float routeFpsDiff = 0.0;
float routeFps = 0.0;
float rotation3 = 0.0;
float cos3 = 0.0;
float lpfCos3 = 0.0;
float lpfCos3Old = 0.0;
float coFps = 0.0;
float targetCoFps = 0.0;
float targetWFps = 0.0;
unsigned long lastHcTime = 0UL;
bool isGettingUp = false;
unsigned long gettingUpStartTime = 0;
bool isGettingDown = false;
unsigned gettingDownStartTime = 0;
struct loc {
  double x;
  double y;
};
struct loc currentLoc;

//short joyX = 0;
//short joyY = 0;
byte stateRun = 0;
byte stateButton1 = 0;
float battVolt = 0.0;
short battRaw = 0;
unsigned long timeMilliseconds = 0UL;
unsigned long timeMicroseconds = 0UL;
bool isRunning = false;
bool isRunReady = false;
bool isUpright = false;
bool isMotorDisable = false;
//bool isHeld = false;
bool isRouteInProgress = false;
bool isHcActive = false;

float sendBatt = 0.0;
float sendFps = 0.0;
unsigned short sendStatus = 0;

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

#define MSG_SIZE 100
char message[MSG_SIZE] = "";

/*******************************************************************************
 * setup()
 ******************************************************************************/
void setup() {
  Serial.begin(115200);
  XBEE_SER.begin(57600);   // XBee, See bottom of this page for settings.
  // Motor pins are initialized in motorInit()
  pinMode(BATT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ACCEL_INTR_PIN, INPUT);
  pinMode(GYRO_INTR_PIN, INPUT);
  pinMode(LED_GN_PIN, OUTPUT);
  pinMode(LED_BU_PIN, OUTPUT);
  pinMode(LED_YE_PIN, OUTPUT);
  pinMode(LED_RE_PIN, OUTPUT);
  pinMode(SW_GN_PIN, INPUT_PULLUP);
  pinMode(SW_BU_PIN, INPUT_PULLUP);
  pinMode(SW_YE_PIN, INPUT_PULLUP);
  pinMode(SW_RE_PIN, INPUT_PULLUP);
    
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(LED_GN_PIN, LOW);
  digitalWrite(LED_BU_PIN, LOW);
  digitalWrite(LED_YE_PIN, LOW);
  digitalWrite(LED_RE_PIN, LOW);

  imuInit();
  motorInit();
  Serial.send_now();
}



/*******************************************************************************
 * loop()
 ******************************************************************************/
void loop() {
  run();
//  testFps();
}


void testFps() {
  unsigned long trigger = 0;
  isRunning = true;
  isMotorDisable = false;

  while(true) {
    readXBee();
    isRunning = isRunReady;
    if (millis() > trigger) {
      trigger = millis() + 200;
//  Serial.print(wFpsLeft); Serial.print("\t");Serial.print(wFpsRight); Serial.print("\t");Serial.println(wFps);
      int pw = abs(joyY);
      Serial.print(isRunning); Serial.print(isMotorDisable); Serial.print(" ");
      Serial.print(pw); Serial.print("\t"); Serial.println(joyY);
      setMotorRight(pw, joyY > 0);
      setMotorLeft(pw, joyY > 0);
    }
  }
}
