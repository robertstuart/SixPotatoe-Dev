#include <Wire.h>
#include "common.h"
#include <digitalWriteFast.h>

#define BATT_PIN        A0
#define DIR_RIGHT_PIN    7
#define DIR_LEFT_PIN     8
#define PWM_RIGHT_PIN    9
#define PWM_LEFT_PIN    10
#define LED_PIN         13
#define ENC_A_RIGHT_PIN  2
#define ENC_A_LEFT_PIN   3
#define ENC_B_RIGHT_PIN  4
#define ENC_B_LEFT_PIN   5

const float ENC_FACTOR = 381.7f;  // Change pulse width to fps speed, 1/29 gear
const long ENC_FACTOR_M = 3817000L;  // Change pulse width to milli-fps speed

volatile char piReceivedMessage[READ_BUF_SIZE];
volatile bool isPiMessage = false;

boolean isXBee = false;  // true if XBEE connect to serial rather than to computer via usb

int xBeeMsgType = 0;
short joyX = 22;
short joyY = 33;
byte stateRun = 0;
byte stateButton1 = 0;
float battVolt = 0.0;
short battRaw = 0;
unsigned long timeMilliseconds = 0UL;
unsigned long timeMicroseconds = 0UL;
boolean isRunning = false;

volatile long tickPositionRight = 0;
volatile long tickPositionLeft = 0;
volatile long tickTimeRight = 0;
volatile long tickTimeLeft = 0;
volatile long tickSumRight = 0;
volatile long tickSumLeft = 0;
volatile long tickCountRight = 0;
volatile long tickCountLeft = 0;

volatile int writeCount = 0;
char sendPacket[WRITE_BUF_SIZE] = {1,2,3,4,5,6,7,8,9,10,11};

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
//  Wire.setClock(400000);   // Needed?
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.begin(57600);  // For XBee
}



/*******************************************************************************
 * loop()
 ******************************************************************************/
void loop() {
  static unsigned long t = 0;
  timeMicroseconds = micros();
  timeMilliseconds = millis();
//  readXBee();
  if (isPiMessage) {
    isPiMessage = false;
    parsePiMessage();
    checkMotors();
    prepareSendPacket();
    batt();
    blinkLed();
  }
}



/*******************************************************************************
 * parsePiMessage()           
 ******************************************************************************/
void parsePiMessage() {
  short r = (piReceivedMessage[0] & 0XFF) << 8;
  r |= (piReceivedMessage[1] & 0XFF);
  targetWhFpsRight = ((float) r) * 0.01;
  
  short l = (piReceivedMessage[1] & 0XFF) << 8;
  l |= (piReceivedMessage[1] && 0XFF);
  targetWhFpsRight = ((float) l) * 0.01;

  xBeeMsgType = piReceivedMessage[4];
  short v = (piReceivedMessage[5] & 0xFF) << 8;
  v |= (piReceivedMessage[6] & 0xFF);
}


/*******************************************************************************
 * blinkLed()  Blinking indicates packets being received.         
 ******************************************************************************/
void blinkLed() {
  static int count = 0;
  static int toggle = false;
  if ((++count % 20) == 0) {
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

