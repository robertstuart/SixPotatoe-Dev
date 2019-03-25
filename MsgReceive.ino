
const int UP_BUFFER_SIZE = 100;
char msgStrUp[UP_BUFFER_SIZE];
int msgStrPtrUp = 0;
int msgCmdUp = 0;
boolean isMessageInProgressUp = false;

//char msgStrX[B_BUFFER_SIZE];
//int msgStrPtrX = 0;
//boolean isMessageInProgressX = false;



/******************************************************************************
    readUp()
 *****************************************************************************/
void readUp() {
  while (Serial.available()) {
    byte b = Serial.read();
    if (b >= 128) {
      msgStrPtrUp = 0;
      msgCmdUp = b;
      isMessageInProgressUp = true;
    } else {
      if (isMessageInProgressUp) {
        if (msgStrPtrUp >= UP_BUFFER_SIZE) {
          isMessageInProgressUp = false;
        } else if (b == 0) {
          msgStrUp[msgStrPtrUp] = 0;
          doUpMsg(msgCmdUp, msgStrUp, msgStrPtrUp);
          isMessageInProgressUp = false;
        } else {
          msgStrUp[msgStrPtrUp++] = b;
        }
      } else {
        Serial.print("Up serial error: "); Serial.println(b);
      }
    }
  }
}



const int DATA_FRAME_MAX = 72;
byte rcvDataFrame[DATA_FRAME_MAX + 1];
int rcvDataFramePtr = 0;
int rcvPacketCount = 0;
int rcvDataFrameLength = 0;
/******************************************************************************
 * readXBee()  XBee. Read bytes from the XBee radio, and call
 *     interpretDataFrame() whenever there is a complete
 *     data packet.
 *****************************************************************************/
void readXBee() {
  static boolean escState = false;

  while (XBEE_SER.available() > 0) {
    byte b = XBEE_SER.read();
    if (b == 0x7e) {
      rcvPacketCount = 0;
      rcvDataFramePtr = 0;
      escState = false;
    } else {
      if (escState) {
        b = b ^ 0x20;
        escState = false;
      }
      else if (b == 0x7D) {
        escState = true;
        return;
      }

      if (rcvPacketCount == 1) rcvDataFrameLength = b * 256;
      else if (rcvPacketCount == 2) rcvDataFrameLength += b;
      else {
        if (rcvDataFramePtr < rcvDataFrameLength) {
          rcvDataFrame[rcvDataFramePtr++] = b;
        } else if (rcvDataFramePtr == rcvDataFrameLength) { // Checksum
          interpretRcvDataFrame();
          rcvDataFramePtr++;  // just in case...
        }
      }
    }
    rcvPacketCount++;
  }
}  // end readXBee()



/******************************************************************************
   interpretRcvDataFrame()  XBee.
 *****************************************************************************/
void interpretRcvDataFrame() {
  switch (rcvDataFrame[0]) { // cmdID
    case 0x8A:           // Modem Status
      //      Serial.println(rcvDataFrame[4], HEX);
      break;
    case 0x88:           // AT Command Response
      break;
    case 0x97:           // Remote Command Response
      break;
    case 0x8B:           // Transmit Status
      break;
    case 0x90:           // Receive Packet (A0=0)
      doRFData();
      break;
    case 0x91:           // Receive Packet (A)=1)
      break;
    default:
      break;
  }
}



/******************************************************************************
 *  doRFData()  XBee
 *****************************************************************************/
void doRFData() {
  static int cmd;
  int rfPtr = 12;
  char msgVal[100];
  int msgValPtr = 0;
  while (rfPtr < rcvDataFrameLength) {
    byte b = rcvDataFrame[rfPtr];
    if (b < 128) {
      msgVal[msgValPtr++] = b;
    }
    if ((b > 127) || (rfPtr == (rcvDataFrameLength - 1))) {
      if (msgValPtr > 0) {
        doHcMsg(cmd, msgVal, msgValPtr, true);
      }
      msgValPtr = 0;
      cmd = b;
    }
    rfPtr++;
  }
  //  Serial.println();
}



/******************************************************************************
 *  doHcMsg() Act on messages from hand controller
 *****************************************************************************/
void doHcMsg(int cmd, char msgStr[], int count, boolean isHc) {
  int intVal;
  float floatVal;
  boolean booleanVal;
  String ss;

  msgStr[count] = 0; // Just to be sure.
//Serial.print(cmd); Serial.print("\t"); Serial.println(msgStr);
  switch (cmd) {
    case RCV_JOYX_I:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        joyX = (float) intVal;
//        if (abs(controllerX) < 0.02) controllerX = 0.0;
      }
      break;
    case RCV_JOYY_I:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        joyY = (float) intVal;
        sendStatusXBeeHc(); // Send status whenever this is recieved.
        if (abs(joyY) < 0.02) joyY = 0.0;
      }
     break;
    case RCV_RUN:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        isRunReady = !isRunReady;
      }
      break;
      break;
    case RCV_V1: // Set this to point to intended variable.
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        float inc = 0.1;
        ACCEL_PITCH_OFFSET = ACCEL_PITCH_OFFSET + ((intVal == 0) ?  -inc :  inc);
        snprintf(message, MSG_SIZE, "Offset: %4.1f", ACCEL_PITCH_OFFSET);\
        sendXMsg(SEND_MESSAGE, message);
        Serial.println(ACCEL_PITCH_OFFSET);
      }
      break;
    case RCV_V2: // Set this to point to intended variable.
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        float inc = 0.1;
//        tp7.z = (intVal == 0) ? tp7.z - inc : tp7.z + inc;
      }
      break;
    case RCV_KILL_UP:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        if (intVal == 0) sendUMsg(TOUP_KILL_UP, 0);
      }
      break;
    case RCV_MOT_DISABLE:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        if (intVal == 0) isMotorDisable = !isMotorDisable;
      }
      break;
    case RCV_GET_UP:
      setGetUp();
      break;
    case RCV_GET_DOWN:
      setGetDown();
      break;
    default:
      Serial.print("Illegal message received: "); Serial.println(cmd);
      break;
  }
}

/******************************************************************************
 *  doUpMsg()
 *****************************************************************************/
void doUpMsg(int cmd, char msgStr[], int count) {
  static float locX = 0.0;
  int intVal;
  float floatVal;
  boolean booleanVal;
  String ss;

  lastHcTime = timeMilliseconds;
  //Serial.println(cmd);
  switch (cmd) {
//    case FRUP_QUERY:
//      sendUMsg(TOUP_X, 2, currentLoc.x);
//      sendUMsg(TOUP_Y, 2, currentLoc.y);
//      sendUMsg(TOUP_HEADING, 2, gcHeading);
//      sendUMsg(TOUP_PITCH, 0, gaPitch);
//      sendUMsg(TOUP_FPS, 2, coFps);
//      sendUMsg(TOUP_DIST, 2, ((float) tickPosition) / TICKS_PER_FOOT);
//      break;
//    case FRUP_SET_LOC_X:
//      if (sscanf(msgStr, "%f", &floatVal) > 0) {
//        locX = floatVal; // Set aside so we can do both a once
//      }
//      break;
//    case FRUP_SET_LOC_Y:
//      if (sscanf(msgStr, "%f", &floatVal) > 0) {
//        currentLoc.x = (double) locX;
//        currentLoc.y = (double) floatVal;
//      }
//      break;
//    case FRUP_SET_HEAD:
//      if (sscanf(msgStr, "%f", &floatVal) > 0) {
//        setHeading(floatVal);
//      }
//      break;
//    case FRUP_FPS_DIFF:
//      if (sscanf(msgStr, "%f", &floatVal) > 0) {
//        routeFpsDiff = floatVal;
//      }
//      break;
//    case FRUP_FPS:
//      if (sscanf(msgStr, "%f", &floatVal) > 0) {
//        routeFps = floatVal;
//      }
//      break;
//    case FRUP_RT_NUM:
//      if (sscanf(msgStr, "%d", &intVal) > 0) {
//
//      }
//      break;
//    case FRUP_RUN_READY:
//      isRunReady = true;
//      break;
//    case FRUP_MSG:
//      Serial.println(msgStr);
//      break;
    case FRUP_STAT:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        isRouteInProgress = (intVal == 0) ? false : true;
        upStatTime = timeMilliseconds;
      }
      break;
    default:
      Serial.print("Illegal Up message: "); Serial.println("cmd");
      break;
  }
}
