



/*******************************************************************************
   prepareSendPacket() Prepare the packet to be sent on the next
                       requestEvent().
 ******************************************************************************/
void prepareSendPacket() {
    static int part = 0;

  long x = tickPositionRight;
  sendPacket[3] = (byte) (x & 0x000000FF);
  x = x >> 8;
  sendPacket[2] = (byte) (x & 0x000000FF);
  x = x >> 8;
  sendPacket[1] = (byte) (x & 0x000000FF);
  x = x >> 8;
  sendPacket[0] = (byte) (x & 0x000000FF);
//  sendPacket[0] = (byte) 1;

  x = tickPositionLeft;
  sendPacket[7] = (byte) (x & 0x000000FF);
  x = x >> 8;
  sendPacket[6] = (byte) (x & 0x000000FF);
  x = x >> 8;
  sendPacket[5] = (byte) (x & 0x000000FF);
  x = x >> 8;
  sendPacket[4] = (byte) (x & 0x000000FF);

  // Send joyx, joyy, batt & run, & button1 alternately
  part = ++part % 5;

  switch (part) {
    case 0: // joyx
      sendPacket[8]  = RCV_JOYX;
      sendPacket[9]  = (byte) ((joyX >> 8) & 0x00FF);
      sendPacket[10] = (byte) (joyX & 0x00FF);
      break;
    case 1: // joyy
      sendPacket[8]  = RCV_JOYY;
      sendPacket[9]  = (byte) ((joyY >> 8) & 0x00FF);
      sendPacket[10] = (byte) (joyY & 0x00FF);
      break;
    case 2: // batt
      sendPacket[8]  = AP_BATT;
      sendPacket[9]  = (byte) ((battRaw >> 8) & 0x00FF);
      sendPacket[10] = (byte) (battRaw & 0x00FF);
      break;
    case 3: // run
      sendPacket[8] = RCV_RUN;
      sendPacket[9] = 0;
      sendPacket[10] = stateRun;
      break;
    case 4: // button1
      sendPacket[8] = RCV_BUTTON1;
      sendPacket[9] = 0;
      sendPacket[10] = stateButton1;
      break;
    default:
      break;
  }

  writeCount = 0;
}



/*******************************************************************************
   receiveEvent() Called on interrupt when i2c byte received
                  Set isNewPiMsg when a complete 8-byte checksummed received.
 ******************************************************************************/
void receiveEvent(int numBytes) {
  static byte readBuf[READ_BUF_SIZE];
  static unsigned int count = 0;

  while (Wire.available()) {
    byte c = Wire.read();
//    Serial.print((int) c); Serial.println();
    readBuf[count++] = c;
    if (count >= READ_BUF_SIZE) {
      int sum = 0;
      for (int i = 0; i < (READ_BUF_SIZE - 1); i++) {
        sum += readBuf[i];
      }
//    Serial.print(sum); Serial.println();
//if (sum == 228) {
//  Serial.print(sum & 0xFF); Serial.print(" "); Serial.println(readBuf[READ_BUF_SIZE]);
//}
      if ((sum & 0xFF) == readBuf[READ_BUF_SIZE - 1]) {              // Correct checksum?
        for (int i = 0; i < (READ_BUF_SIZE - 1); i++) { //     yes
          piReceivedMessage[i] = readBuf[i];
        }
        isPiMessage = true;
        count = 0;
      } else {                                           //     no
        for (int i = 0; i < (READ_BUF_SIZE - 1); i++) {
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
//    delayMicroseconds(5); 
    byte c = sendPacket[writeCount];
    Wire.write(sendPacket,WRITE_BUF_SIZE);
//    if (writeCount < WRITE_BUF_SIZE)
//      writeCount++;
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



