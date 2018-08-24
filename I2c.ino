



/*******************************************************************************
   prepareSendPacket() Prepare the packet to be sent on the next
                       requestEvent().
 ******************************************************************************/
void prepareSendPacket() {
  isPiNeedsNew = false;
  
  noInterrupts();
  long tRight = tickPositionRight;
  long tLeft = tickPositionLeft;
  long sRight = tickSumRight;
  long sLeft = tickSumLeft;
  byte cRight = (byte) tickCountRight;
  byte cLeft = (byte) tickCountLeft;
  long ttRight = tickTimeRight;
  long ttLeft = tickTimeLeft;
  tickSumRight = 0L;
  tickSumLeft = 0L;
  tickCountRight = 0L;
  tickCountLeft = 0L;
  interrupts();

  // Put time since the last tick if there have been no ticks.
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
  putShort(battRaw); // 20
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
   receiveEvent() Called on interrupt when i2c messager received
                  Set isNewPiMsg when a complete 8-byte checksummed received.
 ******************************************************************************/
void receiveEvent(int numBytes) {
  int i = 0;
  while (Wire.available()) {
    if (i < PA_BUF_SIZE) {
      piReceivedMessage[i++] = Wire.read();
    }
  }

  if (numBytes == 1) {
    isPiNeedsNew = true; 
  } else {
    isPiMessage = true;
  }
}



/*******************************************************************************
   requestEvent() Called on interrupt when request for a byte is received.
 ******************************************************************************/
void requestEvent() {
  Wire.write(sendPacket, AP_BUF_SIZE);
}

