



/*******************************************************************************
 * prepareSendPacket() Prepare the packet to be sent on the next
 *                     requestEvent().
 ******************************************************************************/
void prepareSendPacket() {
  static int part = 0;
  byte xBeeCmd = 0;
  short xBeeVal = 0;
  
  part = ++part % 5;
  switch (part) {              // 
    case 0: // joyx
      xBeeCmd = RCV_JOYX_I;
      xBeeVal = joyX;
      break;
    case 1: // joyy
      xBeeCmd = RCV_JOYY_I;
      xBeeVal = joyY;
      break;
    case 2: // batt
      xBeeCmd = AP_BATT;
      xBeeVal = battRaw;
      break;
    case 3: // run
      xBeeCmd = RCV_RUN;
      xBeeVal = ((short) (stateRun == true) ? 1 : 0);
      break;
    case 4: // button1
      xBeeCmd = RCV_BUTTON1;
      xBeeVal = ((short) (stateButton1 == true) ? 1 : 0);
      break;
    default:
      break;
  }

  noInterrupts();
  long tRight = tickPositionRight;
  long tLeft = tickPositionLeft;
  long sRight = tickSumRight;
  long sLeft = tickSumLeft;
  byte cRight = (byte) tickCountRight;
  byte cLeft = (byte) tickCountRight;
  long ttRight = tickTimeRight;
  long ttLeft = tickTimeLeft;
  tickSumRight = 0L;
  tickSumLeft = 0L;
  tickCountRight = 0L;
  tickCountLeft = 0L;
  interrupts();
  
  unsigned long tPoll = micros();
  if (cRight == 0) sRight = tPoll - ttRight;
  if (cLeft == 0) sLeft = tPoll - ttLeft;

  writeCount = 0;
  putLong(tRight);   //  4
  putLong(tLeft);    //  8
  putLong(sRight);   // 12
  putLong(sLeft);    // 16
  putByte(cRight);   // 17
  putByte(cLeft);    // 18
  putByte(xBeeCmd);  // 19
  putShort(xBeeVal); // 21
}

void putLong(long lVal) {
  sendPacket[writeCount++] = (lVal >> 24) & 0x000000FF;
  sendPacket[writeCount++] = (lVal >> 16) & 0x000000FF;
  sendPacket[writeCount++] = (lVal >> 8)  & 0x000000FF;
  sendPacket[writeCount++] = lVal         & 0x000000FF;
}

void putShort(short sVal) {
  sendPacket[writeCount++] = (sVal >> 8) & 0x000000FF;
  sendPacket[writeCount++] = sVal        & 0x000000FF;
}

void putByte(byte bVal) {
  sendPacket[writeCount++] = bVal;
}



/*******************************************************************************
   receiveEvent() Called on interrupt when i2c byte received
                  Set isNewPiMsg when a complete 8-byte checksummed received.
 ******************************************************************************/
void receiveEvent(int numBytes) {
  static int p = 0;
  static byte readBuf[PA_BUF_SIZE];
  static unsigned int count = 0;

  while (Wire.available()) {
    byte c = Wire.read();
//    Serial.print((int) c); Serial.print(' ');
//    if ((++p % 8) == 0) Serial.println();
    readBuf[count++] = c;
    if (count >= PA_BUF_SIZE) {
      int sum = 0;
      for (int i = 0; i < (PA_BUF_SIZE - 1); i++) {
        sum += readBuf[i];
      }
      sum &= 0xFF;
//      Serial.print(sum); Serial.println();
      if (sum == readBuf[PA_BUF_SIZE - 1]) {    // Correct checksum?
        for (int i = 0; i < (PA_BUF_SIZE - 1); i++) {    //     yes
          piReceivedMessage[i] = readBuf[i];
        }
        isPiMessage = true;
        count = 0;
      } else { // Incorrect checksum.  Shift left and try on next character.
        for (int i = 0; i < (PA_BUF_SIZE - 1); i++) {
          readBuf[i] = readBuf[i + 1];
        }
        count--;
      }
    }
  }
}


  /*******************************************************************************
     requestEvent() Called on interrupt when request for a byte is received.
   ******************************************************************************/
  void requestEvent() {
    unsigned long t1, t2;
    
    t2 = micros();
    Wire.write(sendPacket,AP_BUF_SIZE);
    t1 = micros();
    x = t1 - t2;
//    Wire.write(sendPacket,21);
//    Serial.println(timeMilliseconds);
  }



  /*******************************************************************************
     readPacket()
   ******************************************************************************/
  //void readPacket() {
  //}



  /*******************************************************************************
     writePacket()
   ******************************************************************************/
  //void writePacket() {
  //}



