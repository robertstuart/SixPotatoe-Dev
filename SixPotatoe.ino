#include <Wire.h>
//#include <i2c_t3.h>
#include "common.h"
//#include <digitalWriteFast.h>

// Function prototypes
void receiveEvent(size_t count);
void requestEvent(void);

#define BATT_PIN        A0
#define DIR_RIGHT_PIN    6
#define DIR_LEFT_PIN     5
#define PWM_RIGHT_PIN    4
#define PWM_LEFT_PIN     3
#define LED_PIN         13
#define ENC_A_RIGHT_PIN 10
#define ENC_A_LEFT_PIN   8
#define ENC_B_RIGHT_PIN  9
#define ENC_B_LEFT_PIN   7

const float ENC_FACTOR = 381.7f;  // Change pulse width to fps speed, 1/29 gear
const long ENC_FACTOR_M = 3817000L;  // Change pulse width to milli-fps speed

char piReceivedMessage[PA_BUF_SIZE];
volatile bool isPiNeedsNew = false;
volatile bool isPiMessage = false;

short joyX = 0;
short joyY = 0;
byte stateRun = 0;
byte stateButton1 = 0;
float battVolt = 0.0;
short battRaw = 0;
unsigned long timeMilliseconds = 0UL;
unsigned long timeMicroseconds = 0UL;
boolean isRunning = true;

float sendBatt = 0.0;
float sendFps = 0.0;
unsigned short sendStatus = 0;

int bCount = 0;

volatile long tickPositionRight = 0;
volatile long tickPositionLeft = 0;
volatile long tickTimeRight = 0;
volatile long tickTimeLeft = 0;
volatile long tickSumRight = 0;
volatile long tickSumLeft = 0;
volatile long tickCountRight = 0;
volatile long tickCountLeft = 0;

volatile int writeCount = 0;
char sendPacket[AP_BUF_SIZE] = {1,2,3,4,5,6,7,8,9,10,11};

float targetWhFpsRight = 0.0;
float targetWhFpsLeft = 0.0;
long whMFpsRight = 0;
long whMFpsLeft = 0;
float whFpsRight = 0.0;
float whFpsLeft = 0.0;
float whFps = 0.0;

/*******************************************************************************
 * setup()
 ******************************************************************************/
void setup() {
  pinMode(BATT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  
  digitalWrite(DIR_RIGHT_PIN, LOW);
  digitalWrite(DIR_LEFT_PIN, LOW);
  analogWrite(PWM_RIGHT_PIN, 0);
  analogWrite(PWM_LEFT_PIN, 0);
  digitalWrite(LED_PIN, HIGH);

  motorInit();
  Wire.begin(0X1c);
  Wire.setClock(400000);   // Any effect on slave?
//  Wire.begin(I2C_SLAVE, 0X1c, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.begin(115200);
}



/*******************************************************************************
 * loop()
 ******************************************************************************/
void loop() {
  timeMicroseconds = micros();
  timeMilliseconds = millis();
  if (isPiNeedsNew) {
    prepareSendPacket();
  }
  if (isPiMessage) {
    parsePiMessage();
    batt();
    blinkLed();
  }
}



/*******************************************************************************
 * parsePiMessage() New message from Pi received.     
 ******************************************************************************/
void parsePiMessage() {
  isPiMessage = false;
  
  // Right motor direction and pw
  boolean rDir = (piReceivedMessage[0] == 0) ? false : true;
  byte rPw = piReceivedMessage[1];
  setMotorRight(rPw, rDir);

  // Left motor direction and pw
  boolean lDir = (piReceivedMessage[2] == 0) ? false : true;
  byte lPw = piReceivedMessage[3];
  setMotorLeft(lPw, lDir);
}

void debug2(boolean rDir, byte rPw, boolean lDir, byte lPw) {
  static int count = 0;
  if ((++count % 50) == 0) {
    Serial.print(rDir); Serial.print(" "); Serial.print(rPw); Serial.print("\t");
    Serial.print(lDir); Serial.print(" "); Serial.print(lPw); 
    Serial.println();
  }
}

/*******************************************************************************
 * blinkLed()  Blinking indicates packets being received.         
 ******************************************************************************/
void blinkLed() {
  static int bcount = 0;
  static int toggle = false;
  if ((++bcount % 20) == 0) {
    toggle = !toggle;
    digitalWrite(LED_PIN, (toggle) ? HIGH : LOW);
  }
}



/*******************************************************************************
 * batt()  Read battery voltage.    
 ******************************************************************************/
void batt() {
  static unsigned long battTrigger = 0;
  if (timeMilliseconds > battTrigger) {
    battTrigger = timeMilliseconds + 1000;
    battRaw = analogRead(BATT_PIN);
  }
}

void debug() {
  static int dcount = 0;
  if ((++dcount % 100) == 0) {
    Serial.print(tickCountLeft); Serial.print("\t");
    Serial.print(tickCountRight); Serial.print("\t");
    Serial.print(joyX); Serial.print("\t");
    Serial.println();
  }
}

