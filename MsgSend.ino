const int RF_DATA_SIZE = 72;
byte rfData[RF_DATA_SIZE];
int rfDataPtr = 0;

/******************************************************************************
 * sendStatusXBeeHc() Send status to the hand controller.  Sent in 
 *                    response to JOY_Y
 *****************************************************************************/
void sendStatusXBeeHc() {
  static int part = 0;
  part = ++part % 2;
  switch (part) {
    case 0:
      sendXMsg(SEND_FPS, 2, wFps);
      break;
    case 1:
      sendXMsg(SEND_BATT_A, 2, battVolt);
      break;
    default:
      break;
  }
  sendXMsg(SEND_STATE, getState());

  xTransmitRequest(XBEE_DEST_C1, rfData, rfDataPtr);
  rfDataPtr = 0;
}



int getState() {
  int statusInt = 0;
  if (isRunning)          statusInt += 1;
  if (isRunReady)         statusInt += 2;
  if (isUpright)          statusInt += 4;
  if (isRouteInProgress)  statusInt += 8;
  return statusInt;
}



/******************************************************************************
 * send??Msg()
 *****************************************************************************/
void sendXMsg(int cmd, int precision, float val) {  // Float
  char buf[20];
  int len = sprintf(buf, "%.*f", precision, val);
  xAddMessage(cmd, buf, len);
}

void sendXMsg(int cmd, int val) {                   // Integer
  char buf[10];
  int len = sprintf(buf, "%d", val);
  xAddMessage(cmd, buf, len);
}

void sendXMsg(int cmd, String val) {                 // String
  char buf[50];
  int len = val.length();
  if (len >= 50) return;
  val.toCharArray(buf, len + 1);
  xAddMessage(cmd, buf, len);
}

void sendUMsg(int cmd, int precision, double val) {    // Float
      Serial.write((byte) cmd); 
      Serial.print(val, precision); 
      Serial.write((byte) 0);
}

void sendUMsg(int cmd, int val) {                       // Integer
      Serial.write((byte) cmd); 
      Serial.print(val); 
      Serial.write((byte) 0);
}

void sendUMsg(int cmd, int precision, String val) {      // String
      Serial.write((byte) cmd); 
      Serial.print(val); 
      Serial.write((byte) 0);
}



/******************************************************************************
 * xAddMessage()  XBee. Add message to rfData
 *****************************************************************************/
void xAddMessage(int cmd, char buf[], int len) {
  if ((len + 1 + rfDataPtr) >= RF_DATA_SIZE) return;
  rfData[rfDataPtr++] = cmd;
  for (int i = 0; i < len; i++) {
    rfData[rfDataPtr++] = buf[i];
  }
}



/******************************************************************************
 * xTransmitRequest()  XBee. Create a Transmit Request data frame from the 
 *                     rfDataFrame and send it out.
 *****************************************************************************/
void xTransmitRequest(int dest, byte rfFrame[], int rfLength) { 
  static byte txRequestDataFrame[100];
  static int frameId = 0;
  unsigned int sh, sl;
  frameId = ++frameId % 200;   // ID cycles 1-200
  if (dest == XBEE_DEST_C1) {
    sh = XBEE_C1_SH;
    sl = XBEE_C1_SL;
  } else return;
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



/******************************************************************************
 * xTransmitUartFrame()  Xbee. Set all frame bytes and send out with
 *                        appropriate characters escaped.
 *****************************************************************************/
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
  XBEE_SER.write(uartXmitFrame, oPtr);
}
