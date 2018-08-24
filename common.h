#ifndef COMMON_H
#define COMMON_H

const int AP_BUF_SIZE = 20;  // Arduino to Pi buffer
const int PA_BUF_SIZE = 4;   // Pi to Arduino buffer

// Responses from Due to Pi.  All responses are 6 bytes.
const int RESP_MARK       =  0;   // Null response.
const int RESP_RANGE      =  1;   //
const int RESP_BATT       =  2;   // Battery voltage.

const int XBEE_2P_SH         = 0x13A200;    // TwoPotatoe Serial
const int XBEE_2P_SL         = 0x409BD79F;
const int XBEE_3P_SH         = 0x13A200;    // ThreePotatoe Serial
const int XBEE_3P_SL         = 0x409FEBCF;
const int XBEE_C1_SH         = 0x13A200;    // Controller1 Serial
const int XBEE_C1_SL         = 0x409FEBF8;

const int XBEE_DEST_2P       = 0;           // TwoPotatoe destination
const int XBEE_DEST_6P       = 1;           // ThreePotatoe destination
const int XBEE_DEST_C1       = 3;           // Controller 1 destination

// Send from PI to Arduino and Arduino to controller
const int SEND_FPS         = 130;
const int SEND_BATT        = 137;
const int SEND_STATE       = 138;

// Receive from controller to Arduino and Arduino to PI
//const int RCV_JOYX         = 129;
//const int RCV_JOYY         = 130;
const int RCV_RUN          = 131;
const int RCV_BUTTON1      = 132;
const int RCV_JOYX_I       = 160;
const int RCV_JOYY_I       = 161;

// Just Arduino to PI
const int AP_BATT          = 201;

#endif /* COMMON_H */
