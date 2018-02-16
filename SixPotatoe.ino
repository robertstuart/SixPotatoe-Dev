//#include <Wire.h>
#include <i2c_t3.h>
#include "common.h"
//#include <digitalWriteFast.h>

// Function prototypes
void receiveEvent(size_t count);
void requestEvent(void);

//#define BATT_PIN        A0
//#define DIR_RIGHT_PIN    7
//#define DIR_LEFT_PIN     8
//#define PWM_RIGHT_PIN    9
//#define PWM_LEFT_PIN    10
//#define LED_PIN         13
//#define ENC_A_RIGHT_PIN  2
//#define ENC_A_LEFT_PIN   3
//#define ENC_B_RIGHT_PIN  4
//#define ENC_B_LEFT_PIN   5
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
volatile bool isPiMessage = false;

int xBeeMsgType = 0;
short joyX = 22;
short joyY = 33;
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
//  Wire.begin(0X1c);
//  Wire.setClock(400000);   // Any effect on slave?
  Wire.begin(I2C_SLAVE, 0X1c, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial1.begin(57600);  // For XBee
  Serial.begin(115200);
}



/*******************************************************************************
 * loop()
 ******************************************************************************/
void loop() {
  static unsigned long bTrigger = 0;
//  static unsigned long t1 = 0;
//  unsigned long t2 = 0;
  timeMicroseconds = micros();
  timeMilliseconds = millis();
  readXBee();
  if (isPiMessage) {
//    debug();
    isPiMessage = false;
    parsePiMessage();
    prepareSendPacket();
    batt();
    blinkLed();
  }

  unsigned long t = millis();
  if (t > bTrigger) {
    bTrigger = t + 1000;
    Serial.print(bCount); Serial.print(" ");
    Serial.println(t / 1000);
  }
  
}



/*******************************************************************************
 * parsePiMessage()           
 ******************************************************************************/
void parsePiMessage() {
  
  // Right motor direction and pw
  boolean rDir = (piReceivedMessage[0] == 0) ? false : true;
  byte rPw = piReceivedMessage[1];
  setMotorRight(rPw, rDir);

  // Left motor direction and pw
  boolean lDir = (piReceivedMessage[2] == 0) ? false : true;
  byte lPw = piReceivedMessage[3];
  setMotorLeft(lPw, lDir);

  // Message to be sent to Xbee
  byte xBeeCmd = piReceivedMessage[4];
  short xBeeVal = (piReceivedMessage[5] & 0XFF) << 8;
  xBeeVal |= (piReceivedMessage[6] & 0XFF);
  switch(xBeeCmd) {  // Message to be sent to XBee
    case SEND_FPS:
      sendFps = ((float) xBeeVal) * 0.01;
      break;
    case SEND_BATT:
      sendBatt = ((float) xBeeVal) * 0.01;
      break;
    case SEND_STATE:
      sendStatus = xBeeVal;
      break;
  }
//  debug2(rDir, rPw, lDir, lPw);
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
    battVolt = ((float) analogRead(BATT_PIN)) * .022;
    battRaw = (short) (battVolt * 100.0);
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

