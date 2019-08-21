/*****************************************************************************-
 *                        Ma_Hc
 *              Master to Hand Controller - Constants
 *****************************************************************************/

const int XBEE_2P_SH         = 0x13A200;    // TwoPotatoe Serial
const int XBEE_2P_SL         = 0x409BD79F;
const int XBEE_6P_SH         = 0x13A200;    // SixPotatoe Serial
const int XBEE_6P_SL         = 0x409FEBCF;
const int XBEE_C1_SH         = 0x13A200;    // Controller1 Serial
const int XBEE_C1_SL         = 0x409FEBF8;

const int XBEE_DEST_2P       = 0;           // TwoPotatoe destination
const int XBEE_DEST_3P       = 1;           // ThreePotatoe destination
const int XBEE_DEST_C1       = 3;           // Controller 1 destination

// Messages sent from Master to Hand Controller
enum {
  SEND_MESSAGE = 129,
  SEND_FPS,
  SEND_PITCH,
  SEND_HEADING,
  SEND_ROUTE_STEP,
  SEND_STATE,
  SEND_BATT,
  SEND_MODE,
  SEND_ROUTE_NAME,
  SEND_XPOS,
  SEND_YPOS,
  SEND_V1,
  SEND_V2
};

// Messages received, from Hand Controller to Master
enum {
  RCV_JOYX = 129,
  RCV_JOYY,
  RCV_JOYX_I,  // For SixPotatoe
  RCV_JOYY_I, 
  RCV_BUTTON
};

enum {
  BUTTON_1L,
  BUTTON_1M,
  BUTTON_1R,
  BUTTON_2L,
  BUTTON_2M,
  BUTTON_2R,
  BUTTON_3L,
  BUTTON_3M,
  BUTTON_3R,
  BUTTON_4L,
  BUTTON_4M,
  BUTTON_4R,
};

const int IS_PRESS_BIT   = 0x01;
const int IS_SHIFT_BIT     = 0x02;
const int IS_CTRL_BIT      = 0x04;
