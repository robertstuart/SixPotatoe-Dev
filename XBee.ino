//#include "XBee.h"

  const int DATA_FRAME_MAX = 72;
  byte rcvDataFrame[DATA_FRAME_MAX + 1];
  int rcvDataFramePtr = 0;
  int rcvPacketCount = 0;
  int rcvDataFrameLength = 0;
  
  const int RF_DATA_SIZE = 72;
  byte rfData[RF_DATA_SIZE];
  int rfDataPtr = 0;
  byte rf2Data[RF_DATA_SIZE];
  int rf2DataPtr = 0;


/*******************************************************************************
 * readXBee()  Read bytes from the XBee radio, and call
 *             interpretDataFrame() whenever there is a complete
 *             data packet.
 ******************************************************************************/
void readXBee() {
  static boolean escState = false;

  if (!isXBee) return;
  
  while (Serial.available() > 0) {
    byte b = Serial.read();
// Serial.print(b);
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




/**************************************************************************.
 * interpretRcvDataFrame()
 **************************************************************************/
void interpretRcvDataFrame() {
  switch (rcvDataFrame[0]) { // cmdID
    case 0x8A:           // Modem Status
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



/**************************************************************************.
 * doRFData()  Do all of the messages in the RF data packet from 
 *             the hand controller.
 **************************************************************************/
void doRFData() {
  static int cmd;
  int rfPtr = 12;
  char msgVal[100];
  int msgValPtr = 0;
  //Serial.print(rcvDataFrameLength); Serial.print("\t");
  while (rfPtr < rcvDataFrameLength) {
    byte b = rcvDataFrame[rfPtr];
    //Serial.print(b, HEX); Serial.print(" ");
    if (b < 128) {
      msgVal[msgValPtr++] = b;
    }
    if ((b > 127) || (rfPtr == (rcvDataFrameLength - 1))) {
      if (msgValPtr > 0) {
        doMsg(cmd, msgVal, msgValPtr, true);
      }
      msgValPtr = 0;
      cmd = b;
    }
    rfPtr++;
  }
  sendStatusXBeeHc();
}



/*******************************************************************************
 * doMsg() Received a message from the contoller.  Send to Pi.
 ******************************************************************************/
void doMsg(int cmd, char msgStr[], int count, boolean isHc) {
  int intVal;
  float floatVal;
  boolean booleanVal;
  String ss;

  msgStr[count] = 0; // Just to be sure.
//  if ((cmd != RCV_JOYX) && (cmd != RCV_JOYY)) {
    Serial.print(cmd); Serial.print("  "); Serial.println(msgStr);
//  }

  switch (cmd) {
    case RCV_JOYX:
      if (sscanf(msgStr, "%f", &floatVal) >0) {
        joyX = (short) (floatVal * 1000.0);
      }
      break;
    case RCV_JOYY:
      if (sscanf(msgStr, "%f", &floatVal) >0) {
        joyY = (short) (floatVal * 1000.0);}
      break;
    case RCV_RUN:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        stateRun = (intVal) ? 1 : 0;
      }
      break;
    default:
      Serial.print("Illegal message received: "); Serial.println(cmd);
      break;
  }
}



///*******************************************************************************
// * sendXbee() Send status for display on the hand controller.
// *            Only send if it has changed
// ******************************************************************************/
//void sendXBee() {
//  static short oldBattRaw = 0;
//  static short oldFpsRaw = 0;
//  static short oldState = 0;
//
//  if (oldBattRaw != battRaw) {
//    oldBattRaw = battRaw;
//        sendXMsg(SEND_BATT, 2, ((float) battRaw) / 100.0);
//  }
//  if (oldFpsRaw != fpsRaw) {
//    oldFpsRaw = fpsRaw;
//      sendXMsg(SEND_FPS, 1, ((float) fpsRaw) * 0.01);    
//  }
//  if (oldState != state) {
//    oldState = state;
//    sendXMsg(SEND_STATE, state);
//  }
//}


/*******************************************************************************
 * sendXMsg() Send info for display on the hand controller.
 ******************************************************************************/
void sendXMsg(int cmd, int precision, double val) {
  char buf[20];
  int len = sprintf(buf, "%.*f", precision, val);
  xAddMessage(cmd, buf, len);
}

void sendXMsg(int cmd, int val) {
  char buf[10];
  int len = sprintf(buf, "%d", val);
  xAddMessage(cmd, buf, len);
}

void sendXMsg(int cmd, String val) {
  char buf[50];
  int len = val.length();
  if (len >= 50) return;
  val.toCharArray(buf, len + 1);
  xAddMessage(cmd, buf, len);
}


/**************************************************************************.
 * xAddMessage()
 *                Add message to rfData
 **************************************************************************/
void xAddMessage(int cmd, char buf[], int len) {
  if ((len + 1 + rfDataPtr) >= RF_DATA_SIZE) return;
  rfData[rfDataPtr++] = cmd;
  for (int i = 0; i < len; i++) {
    rfData[rfDataPtr++] = buf[i];
  }
}



/**************************************************************************.
 * sendStatusXBeeHc()
 *                Send a reply to hc
 **************************************************************************/
void sendStatusXBeeHc() {
//  static int part = 0;
//  part = ++part % 4;
//  switch (part) {
//    case 0:
//      sendXMsg(SEND_FPS, 2, ((float) fpsRaw) * 0.001);
//      break;
//    case 1:
//      break;
//    case 2:
//       sendXMsg(SEND_BATT, 2, ((float) battRaw) * 0.001);
//     break;
//    case 3:
//      break;
//    default:
//      break;
//  }
//  sendXMsg(SEND_STATE, state);
//
//  xTransmitRequest(XBEE_DEST_C1, rfData, rfDataPtr);
//  rfDataPtr = 0;
}




/**************************************************************************.
 * xTransmitRequest()
 *                    Create a Transmit Request data frame from the 
 *                    rfDataFrame and send it out.
 **************************************************************************/
void xTransmitRequest(int dest, byte rfFrame[], int rfLength) { 
  static byte txRequestDataFrame[100];
  static int frameId = 0;
  unsigned int sh, sl;
  frameId = ++frameId % 200;   // ID cycles 1-200
  sh = XBEE_C1_SH;
  sl = XBEE_C1_SL;
  txRequestDataFrame[0] = 0x10;  // API identifier value
  txRequestDataFrame[1] = frameId + 1;
  txRequestDataFrame[2] = (sh >> 24) & 0x000000FF;
  txRequestDataFrame[3] = (sh >> 16) & 0x000000FF;
  txRequestDataFrame[4] = (sh >> 8) & 0x000000FF;
  txRequestDataFrame[5] = sh & 0x000000FF;
  txRequestDataFrame[6] = (sl >> 24) & 0x000000FF;
  txRequestDataFrame[7] = (sl >> 16) & 0x000000FF;
  txRequestDataFrame[8] = (sl >> 8) & 0x000000FF;
  txRequestDataFrame[9] = sl & 0x000000FF;
  txRequestDataFrame[10] = 0x24;  // 16-bit network address (PAN ID)
  txRequestDataFrame[11] = 0x56;  // 16-bit network address (PAN ID)
  txRequestDataFrame[12] = 0;     // Raduis
  txRequestDataFrame[13] = 0;     // 0ptions

  for (int i = 0; i < rfLength; i++) {
    txRequestDataFrame[i + 14] = rfFrame[i];
  }
  xTransmitUartFrame(txRequestDataFrame, rfLength + 14);  
}



/**************************************************************************.
 *
 * xTransmitUartFrame()
 *
 *     Set all frame bytes and send out with appropriate
 *     characters escaped.
 **************************************************************************/
void xTransmitUartFrame(byte dataFrame[], int dataFrameLength) {
  static byte preEscPack[100];
  static byte uartXmitFrame[200];
  int sum = 0;

  // Compute the checksum.
  for (int i = 0; i < dataFrameLength; i++) {
    sum += dataFrame[i];
  }  
  byte checkSum = 0xFF - (sum & 0xFF);

  // Fill out preEscPack with the array that must be escaped. That is, minus FE.
  preEscPack[0] = 0;
  preEscPack[1] = dataFrameLength;
  for (int i = 0; i < dataFrameLength; i++) {
    preEscPack[i + 2] = dataFrame[i];
  }
  preEscPack[dataFrameLength + 2] = checkSum;

  // Now put it into a single array with the escaped characters.
  int oPtr = 1;
  uartXmitFrame[0] = 0x7E;
  for (int i = 0; i < (dataFrameLength + 3); i++) {
    int b = preEscPack[i];
    if ((b == 0x7E) || (b == 0x7D) || (b == 0x11) || (b == 0x13)) {
      uartXmitFrame[oPtr++] = 0x7D;
      uartXmitFrame[oPtr++] = b ^ 0x20;
    }
    else {
      uartXmitFrame[oPtr++] = b;
    }
  }
//  XBEE_SER.write(uartXmitFrame, oPtr);
}


